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
            filename = f"/root/catkin_ws/performance_results_{timestamp}.csv"
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
            rospy.sleep(1 / speed)

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
