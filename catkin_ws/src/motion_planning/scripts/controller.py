import math
import copy
import rospy
import numpy as np
import kinematics
import control_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
from pyquaternion import Quaternion
import pandas as pd
import datetime
import time
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
import sensor_msgs.msg

# Force update check
rospy.loginfo("Loading controller module...")


def get_controller_state(controller_topic, timeout=None):
    return rospy.wait_for_message(
        f"{controller_topic}/state",
        control_msgs.msg.JointTrajectoryControllerState,
        timeout=timeout,
    )


class ArmController:
    def __init__(self, gripper_state=0, controller_topic="/trajectory_controller"):
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.gripper_state = gripper_state

        self.controller_topic = controller_topic
        self.default_joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        self.default_joint_trajectory.joint_names = self.joint_names

        joint_states = get_controller_state(controller_topic).actual.positions
        x, y, z, rot = kinematics.get_pose(joint_states)
        self.gripper_pose = (x, y, z), Quaternion(matrix=rot)

        # State tracking for logging
        self.current_joint_state = None
        self.state_sub = rospy.Subscriber(
            f"{self.controller_topic}/state",
            control_msgs.msg.JointTrajectoryControllerState,
            self._state_cb,
        )

        # Create an action client for the joint trajectory
        self.joints_pub = rospy.Publisher(
            f"{self.controller_topic}/command",
            trajectory_msgs.msg.JointTrajectory,
            queue_size=10,
        )

        # Logging
        self.log_data = []
        rospy.on_shutdown(self.save_logs)

    def _state_cb(self, msg):
        self.current_joint_state = msg

    def save_logs(self):
        if not self.log_data:
            return
        try:
            df = pd.DataFrame(self.log_data)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"/root/catkin_ws/performance_results_arm_{timestamp}.csv"
            df.to_csv(filename, index=False)
            rospy.loginfo(f"Performance Data saved to {filename}")
            self.log_data = []
        except Exception as e:
            rospy.logerr(f"Failed to save logs: {e}")

    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        (sx, sy, sz), start_quat = self.gripper_pose

        tx, ty, tz = sx + dx, sy + dy, sz + dz
        target_quat = start_quat * delta_quat

        self.move_to(tx, ty, tz, target_quat, blocking=blocking)

    def move_to(
        self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True
    ):
        """
        Move the end effector to target_pos with target_quat as orientation
        :param x:
        :param y:
        :param z:
        :param start_quat:
        :param target_pos:
        :param target_quat:
        :param z_raise:
        :param blocking:
        :return:
        """

        def smooth(percent_value, period=math.pi):
            return (1 - math.cos(percent_value * period)) / 2

        (sx, sy, sz), start_quat = self.gripper_pose

        if x is None:
            x = sx
        if y is None:
            y = sy
        if z is None:
            z = sz
        if target_quat is None:
            target_quat = start_quat

        dx, dy, dz = x - sx, y - sy, z - sz
        length = math.sqrt(dx**2 + dy**2 + dz**2) * 300 + 80
        speed = length

        steps = int(length)
        step = 1 / steps

        for i in np.arange(0, 1 + step, step):
            i_2 = smooth(i, 2 * math.pi)  # from 0 to 1 to 0
            i_1 = smooth(i)  # from 0 to 1

            grip = Quaternion.slerp(start_quat, target_quat, i_1)

            # Calculate intermediate targets
            tx = sx + i_1 * dx
            ty = sy + i_1 * dy
            tz = sz + i_1 * dz + i_2 * z_raise

            t_start = time.time()
            self.send_joints(
                tx,
                ty,
                tz,
                grip,
                duration=1 / speed * 0.9,
            )
            comp_time = time.time() - t_start

            self._record_state(tx, ty, tz, grip, comp_time)
            try:
                rospy.sleep(1 / speed)
            except rospy.exceptions.ROSInterruptException:
                return

        if blocking:
            self.wait_for_position(tol_pos=0.005, tol_vel=0.08)

        self.gripper_pose = (x, y, z), target_quat

    def _record_state(self, tx, ty, tz, target_quat, comp_time=0.0):
        if self.current_joint_state:
            try:
                ax, ay, az, rot = kinematics.get_pose(
                    self.current_joint_state.actual.positions
                )
                actual_quat = Quaternion(matrix=rot)
                error_orient = Quaternion.distance(target_quat, actual_quat)

                data = {
                    "time": rospy.Time.now().to_sec(),
                    "target_x": tx,
                    "target_y": ty,
                    "target_z": tz,
                    "actual_x": ax,
                    "actual_y": ay,
                    "actual_z": az,
                    "error_dist": np.sqrt(
                        (tx - ax) ** 2 + (ty - ay) ** 2 + (tz - az) ** 2
                    ),
                    "error_orient": error_orient,
                    "computation_time": comp_time,
                }

                # Log joint velocities if available
                if (
                    hasattr(self.current_joint_state.actual, "velocities")
                    and len(self.current_joint_state.actual.velocities) > 0
                ):
                    for i, v in enumerate(self.current_joint_state.actual.velocities):
                        data[f"v{i+1}"] = v

                self.log_data.append(data)
            except Exception:
                pass

    def send_joints(
        self, x, y, z, quat, duration=1.0
    ):  # x,y,z and orientation of lego block
        # Solve for the joint angles, select the 5th solution
        joint_states = kinematics.get_joints(x, y, z, quat.rotation_matrix)

        traj = copy.deepcopy(self.default_joint_trajectory)

        for _ in range(0, 2):
            pts = trajectory_msgs.msg.JointTrajectoryPoint()
            pts.positions = joint_states
            pts.velocities = [0, 0, 0, 0, 0, 0]
            pts.time_from_start = rospy.Time(duration)
            # Set the points to the trajectory
            traj.points = [pts]
            # Publish the message
            self.joints_pub.publish(traj)

    def wait_for_position(self, timeout=2, tol_pos=0.01, tol_vel=0.01):
        end = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < end:
            msg = get_controller_state(self.controller_topic, timeout=10)
            v = np.sum(np.abs(msg.actual.velocities), axis=0)
            if v < tol_vel:
                for actual, desired in zip(msg.actual.positions, msg.desired.positions):
                    if abs(actual - desired) > tol_pos:
                        break
                    return
        rospy.logwarn("Timeout waiting for position")

    def sync_state(self):
        """Updates the internal gripper pose from the actual robot state."""
        if self.current_joint_state:
            x, y, z, rot = kinematics.get_pose(
                self.current_joint_state.actual.positions
            )
            self.gripper_pose = (x, y, z), Quaternion(matrix=rot)


class MPCController:
    def __init__(self, velocity_topic="/joint_group_vel_controller/command"):
        self.velocity_topic = velocity_topic
        self.vel_pub = rospy.Publisher(
            self.velocity_topic, Float64MultiArray, queue_size=1
        )
        self.switch_srv = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController
        )
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.current_joints = None
        self.state_sub = rospy.Subscriber(
            "/joint_states", sensor_msgs.msg.JointState, self._state_cb
        )
        self.log_data = []
        rospy.on_shutdown(self.save_logs)

        # PID Gains for MPC
        self.Kp = 15.0  # Increased for faster convergence
        self.Ki = 1.5  # Increased to eliminate steady-state error (<2mm)
        self.Kd = 0.6  # Adjusted to prevent overshoot with higher P

        # State tracking
        self.integral_error = np.zeros(6)
        self.prev_error = np.zeros(6)
        self.prev_time = None

    def _state_cb(self, msg):
        try:
            idxs = [msg.name.index(jn) for jn in self.joint_names]
            self.current_joints = [msg.position[i] for i in idxs]
        except ValueError:
            pass

    def switch_to_mpc(self):
        """Switches from trajectory controller to velocity controller"""
        try:
            self.switch_srv(
                start_controllers=["joint_group_vel_controller"],
                stop_controllers=["trajectory_controller"],
                strictness=1,
            )
            rospy.loginfo("[MPC] Switched to velocity control")
            rospy.sleep(0.1)  # Brief pause for controller to activate
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def switch_to_arm_controller(self):
        """Switches back to standard trajectory controller"""
        try:
            self.switch_srv(
                start_controllers=["trajectory_controller"],
                stop_controllers=["joint_group_vel_controller"],
                strictness=1,
            )
            rospy.loginfo("[MPC] Switched back to trajectory control")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def stop(self):
        """Sends zero velocity to stop the robot"""
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.vel_pub.publish(msg)

    def save_logs(self):
        if not self.log_data:
            return
        try:
            df = pd.DataFrame(self.log_data)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"/root/catkin_ws/performance_results_mpc_{timestamp}.csv"
            df.to_csv(filename, index=False)
            rospy.loginfo(f"MPC Performance Data saved to {filename}")
            self.log_data = []
        except Exception as e:
            rospy.logerr(f"Failed to save MPC logs: {e}")

    def _record_state(self, target_joints, vel):
        if self.current_joints:
            try:
                ax, ay, az, rot = kinematics.get_pose(self.current_joints)
                tx, ty, tz, _ = kinematics.get_pose(target_joints)

                data = {
                    "time": rospy.Time.now().to_sec(),
                    "target_x": tx,
                    "target_y": ty,
                    "target_z": tz,
                    "actual_x": ax,
                    "actual_y": ay,
                    "actual_z": az,
                    "error_x": tx - ax,
                    "error_y": ty - ay,
                    "error_z": tz - az,
                    "error_norm": np.sqrt(
                        (tx - ax) ** 2 + (ty - ay) ** 2 + (tz - az) ** 2
                    ),
                }
                # Add joint velocities
                for i, v in enumerate(vel[:6]):
                    data[f"joint_vel_{i+1}"] = v

                self.log_data.append(data)
            except Exception as e:
                rospy.logwarn(f"Failed to record MPC state: {e}")

    def move_to_mpc(self, x, y, z, target_quat, duration=3.0, tolerance=0.005):
        """Executes a move using MPC velocity control"""
        self.switch_to_mpc()

        # Reset PID states
        self.integral_error = np.zeros(6)
        self.prev_error = np.zeros(6)
        self.prev_time = rospy.Time.now()

        # Get target joints
        target_joints = kinematics.get_joints(x, y, z, target_quat.rotation_matrix)
        if target_joints is None:
            rospy.logerr("[MPC] No IK solution found")
            self.switch_to_arm_controller()
            return False

        rospy.loginfo(f"[MPC] Moving to target: ({x:.3f}, {y:.3f}, {z:.3f})")

        rate = rospy.Rate(100)
        start_time = rospy.Time.now()
        success = False

        while (rospy.Time.now() - start_time).to_sec() < duration:
            if self.current_joints is None:
                rate.sleep()
                continue

            # Calculate time step
            current_time = rospy.Time.now()
            dt = (current_time - self.prev_time).to_sec()
            if dt <= 0:
                dt = 0.01
            self.prev_time = current_time

            # PID Control
            error = np.array(target_joints) - np.array(self.current_joints)

            # Handle joint wrapping (-pi to pi)
            for i in range(len(error)):
                while error[i] > np.pi:
                    error[i] -= 2 * np.pi
                while error[i] < -np.pi:
                    error[i] += 2 * np.pi

            # Check if target reached
            error_norm = np.linalg.norm(error)
            if error_norm < tolerance:
                rospy.loginfo(f"[MPC] Target reached with error: {error_norm:.4f}")
                success = True
                break

            # PID Control
            # Proportional
            p_term = self.Kp * error

            # Integral (with anti-windup)
            self.integral_error += error * dt
            # Clamp integral term
            max_integral = 1.0
            self.integral_error = np.clip(
                self.integral_error, -max_integral, max_integral
            )
            i_term = self.Ki * self.integral_error

            # Derivative
            derivative = (error - self.prev_error) / dt if dt > 0 else np.zeros(6)
            d_term = self.Kd * derivative
            prev_error = error

            # Combine terms
            vel = p_term + i_term + d_term

            # Velocity limits (UR5 safe limits)
            vel = np.clip(vel, -1.5, 1.5)

            # Publish velocity command
            msg = Float64MultiArray()
            msg.data = vel.tolist()
            self.vel_pub.publish(msg)

            # Record state for logging
            self._record_state(target_joints, vel)

            # Log progress every second
            if int((current_time - start_time).to_sec()) != int(
                (self.prev_time - start_time).to_sec()
            ):
                rospy.loginfo(f"[MPC] Progress: Error norm = {error_norm:.4f}")

            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException:
                rospy.loginfo("[MPC] Interrupted by user")
                break

        # Stop robot
        self.stop()
        self.switch_to_arm_controller()

        if not success:
            rospy.logwarn(f"[MPC] Timeout - final error: {error_norm:.4f}")

        return success
