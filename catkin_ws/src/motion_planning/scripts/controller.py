import math
import copy
import rospy
import actionlib
import numpy as np
import kinematics
import sensor_msgs.msg
import std_msgs.msg
import trajectory_msgs.msg
import control_msgs.msg
from pyquaternion import Quaternion
import pandas as pd
import datetime
import time
import warnings
import os
import glob

warnings.filterwarnings("ignore")

rospy.loginfo("Loading controller module...")


class MPCController:
    """
    MPC Controller for UR5 Robot.
    - Uses MPC for intelligent trajectory planning (quintic interpolation)
    - Sends trajectory commands to trajectory_controller via FollowJointTrajectory Action
    """

    def __init__(self, controller_topic="/trajectory_controller"):
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # ===== MPC Parameters =====
        self.dt = 0.02  # 20ms control cycle (50 Hz)
        self.v_max = 2.5  # Joint velocity limit (rad/s)
        self.tracking_gain = 0.9  # P-gain for position tracking

        # Cartesian speed limits
        self.max_cart_speed = 1.0  # m/s (faster but smooth)
        self.error_threshold = 0.003  # 3mm

        # ===== State Variables =====
        self.current_joint_positions = np.zeros(6)
        self.current_joint_velocities = np.zeros(6)
        self.gripper_pose = ((0, 0, 0), Quaternion(1, 0, 0, 0))
        self._state_received = False
        self._target_positions = np.zeros(6)
        self._start_positions = np.zeros(6)
        self._trajectory_start_time = None
        self._trajectory_duration = 0.0
        self._is_moving = False

        # ===== ROS Setup =====
        # Action Client for trajectory execution
        self.client = actionlib.SimpleActionClient(
            f"{controller_topic}/follow_joint_trajectory",
            control_msgs.msg.FollowJointTrajectoryAction,
        )

        self.state_sub = rospy.Subscriber(
            "/joint_states", sensor_msgs.msg.JointState, self._joint_state_cb
        )

        # Wait for first state
        rospy.loginfo("MPC: Waiting for joint states...")
        start_time = time.time()
        while not self._state_received and not rospy.is_shutdown():
            if time.time() - start_time > 5.0:
                rospy.logerr("MPC: Timeout waiting for joint states!")
                break
            rospy.sleep(0.05)
        rospy.loginfo("MPC: Joint states received. Controller ready.")

        # Check for action server
        if not self.client.wait_for_server(timeout=rospy.Duration(2.0)):
            rospy.logwarn(
                "MPC: Trajectory action server not found! Is the controller running?"
            )

        # ===== Logging =====
        self.log_data = []
        rospy.on_shutdown(self.save_logs)
        rospy.loginfo(
            f"MPC Controller Initialized (dt={self.dt}s, v_max={self.v_max}rad/s)"
        )

    def _joint_state_cb(self, msg):
        """Update joint state from /joint_states topic."""
        try:
            # Build lookup maps
            pos_map = {name: pos for name, pos in zip(msg.name, msg.position)}
            vel_map = {
                name: (msg.velocity[i] if i < len(msg.velocity) else 0.0)
                for i, name in enumerate(msg.name)
            }

            # Extract UR5 joints
            self.current_joint_positions = np.array(
                [pos_map[n] for n in self.joint_names]
            )
            self.current_joint_velocities = np.array(
                [vel_map[n] for n in self.joint_names]
            )

            # Update FK (gripper pose)
            x, y, z, rot = kinematics.get_pose(self.current_joint_positions)
            self.gripper_pose = ((x, y, z), Quaternion(matrix=rot))
            self._state_received = True

        except (KeyError, IndexError) as e:
            rospy.logwarn(f"MPC: State update error: {e}")

    def move(self, dx=0, dy=0, dz=0, delta_quat=Quaternion(1, 0, 0, 0), blocking=True):
        """Relative movement in end-effector frame."""
        (sx, sy, sz), start_quat = self.gripper_pose
        tx, ty, tz = sx + dx, sy + dy, sz + dz
        target_quat = start_quat * delta_quat
        self.move_to(tx, ty, tz, target_quat, blocking=blocking)

    def _solve_ik(self, pos, quat, seed_q=None):
        """Helper to solve IK and return joint configuration."""
        try:
            # Compute desired transformation matrix
            desired_T = np.identity(4)
            desired_T[0:3, 0:3] = quat.rotation_matrix
            desired_T[0, 3] = pos[0]
            desired_T[1, 3] = pos[1]

            # Adjust Z for base offset (World -> Base frame)
            # Value taken from kinematics.py get_joints/get_pose
            z_offset = 0.771347
            desired_T[2, 3] = pos[2] - z_offset

            # Solve inverse kinematics
            ik_result = kinematics.inverse(desired_T)
            if ik_result is None or ik_result.size == 0:
                return None

            # Find solution closest to seed (or current) joint positions
            ref_joints = seed_q if seed_q is not None else self.current_joint_positions
            diff = ik_result.T - ref_joints
            dists = np.linalg.norm(diff, axis=1)
            best_idx = np.argmin(dists)
            q_target = ik_result[:, best_idx]

            return q_target
        except Exception as e:
            rospy.logwarn(f"MPC: Error in IK: {e}")
            return None

    def move_to(
        self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True
    ):
        """
        Move end-effector to target pose using MPC-planned trajectory
        with position controller execution.
        """
        (sx, sy, sz), start_quat = self.gripper_pose
        if x is None:
            x = sx
        if y is None:
            y = sy
        if z is None:
            z = sz
        if target_quat is None:
            target_quat = start_quat

        target_pos = np.array([x, y, z])
        start_pos = np.array([sx, sy, sz])

        # ===== ESTIMATE MOVEMENT TIME =====
        dist = np.linalg.norm(target_pos - start_pos)
        duration = max(0.5, dist / self.max_cart_speed)

        rospy.loginfo(
            f"MPC: Planning trajectory to [{x:.4f}, {y:.4f}, {z:.4f}] ({dist*100:.1f}cm, duration={duration:.2f}s)"
        )

        # ===== PLAN TRAJECTORY USING MPC (QUINTIC INTERPOLATION) =====
        num_steps = max(int(duration / self.dt), 5)
        waypoints = []

        for i in range(num_steps + 1):
            t_norm = i / max(1, num_steps)

            # Smooth quintic interpolation: s(t) = t^3(10 - 15t + 6t^2)
            s = t_norm**3 * (10 - 15 * t_norm + 6 * t_norm**2)

            # Interpolate position
            pos = start_pos + s * (target_pos - start_pos)

            # Interpolate orientation (SLERP)
            quat = Quaternion.slerp(start_quat, target_quat, s)

            waypoints.append((pos, quat, t_norm * duration))

        # ===== EXECUTE TRAJECTORY VIA POSITION COMMANDS =====
        rospy.loginfo(f"MPC: Executing trajectory with {len(waypoints)} waypoints")

        # Prepare trajectory points
        points = []
        last_q = self.current_joint_positions

        for pos, quat, t in waypoints:
            q_target = self._solve_ik(pos, quat, seed_q=last_q)
            if q_target is None:
                rospy.logerr("MPC: IK failed during planning. Aborting trajectory.")
                return
            last_q = q_target
            points.append((q_target, t))

        # Velocity Scaling: Check if trajectory violates joint limits
        max_vel_ratio = 0.0
        for i in range(1, len(points)):
            q_curr, t_curr = points[i]
            q_prev, t_prev = points[i - 1]
            dt = t_curr - t_prev
            if dt > 1e-6:
                vel = np.abs(q_curr - q_prev) / dt
                max_v = np.max(vel)
                if max_v > self.v_max:
                    max_vel_ratio = max(max_vel_ratio, max_v / self.v_max)

        time_scale = 1.0
        if max_vel_ratio > 1.0:
            time_scale = max_vel_ratio * 1.1  # Add 10% buffer
            rospy.loginfo(
                f"MPC: Scaling trajectory time by {time_scale:.2f}x to satisfy joint limits"
            )

        # Create Goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names

        for q, t in points:
            pt = trajectory_msgs.msg.JointTrajectoryPoint()
            pt.positions = q.tolist()
            pt.time_from_start = rospy.Time(t * time_scale)
            goal.trajectory.points.append(pt)

        # ===== WAIT FOR MOTION TO COMPLETE =====
        if blocking:
            self.client.send_goal_and_wait(goal)
            self._record_state(x, y, z, 0.0)
        else:
            self.client.send_goal(goal)

        # Report final error
        (cx, cy, cz), c_quat = self.gripper_pose
        dist_error = np.linalg.norm(target_pos - np.array([cx, cy, cz]))

        if dist_error < self.error_threshold:
            rospy.loginfo(f"MPC: Target reached (error={dist_error*1000:.2f}mm)")
        else:
            rospy.logwarn(f"MPC: Final error={dist_error*1000:.2f}mm")

    def execute_recorded_trajectory(self, csv_path=None):
        """
        Replays a trajectory from a CSV file (e.g. generated by ArmController).
        If csv_path is None, finds the latest performance_results_*.csv.
        """
        if csv_path is None:
            # Find latest performance_results csv
            search_dir = "/root/catkin_ws/"
            files = glob.glob(os.path.join(search_dir, "performance_results_*.csv"))
            if not files:
                rospy.logwarn("MPC: No recorded trajectory files found.")
                return
            csv_path = max(files, key=os.path.getctime)

        if not os.path.exists(csv_path):
            rospy.logerr(f"MPC: File not found: {csv_path}")
            return

        rospy.loginfo(f"MPC: Loading trajectory from {csv_path}")
        try:
            df = pd.read_csv(csv_path)

            # Check required columns
            required = ["time", "target_x", "target_y", "target_z"]
            if not all(col in df.columns for col in required):
                rospy.logerr(f"MPC: CSV missing required columns: {required}")
                return

            # Normalize time
            df["time"] = df["time"] - df["time"].iloc[0]

            # Get start orientation
            _, start_quat = self.gripper_pose

            rospy.loginfo(f"MPC: Replaying {len(df)} points...")

            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = self.joint_names

            last_q = self.current_joint_positions

            for i, row in df.iterrows():
                target_pos = np.array(
                    [row["target_x"], row["target_y"], row["target_z"]]
                )

                # Solve IK with continuity
                q_target = self._solve_ik(target_pos, start_quat, seed_q=last_q)

                if q_target is not None:
                    last_q = q_target
                    pt = trajectory_msgs.msg.JointTrajectoryPoint()
                    pt.positions = q_target.tolist()
                    pt.time_from_start = rospy.Time(row["time"])
                    goal.trajectory.points.append(pt)

            self.client.send_goal_and_wait(goal)
            rospy.loginfo(f"MPC: Trajectory execution finished.")

            rospy.loginfo("MPC: Trajectory replay complete.")

        except Exception as e:
            rospy.logerr(f"MPC: Error replaying trajectory: {e}")

    def save_logs(self):
        """Save performance logs to CSV."""
        if not self.log_data:
            return
        try:
            df = pd.DataFrame(self.log_data)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"/root/catkin_ws/mpc_results_{timestamp}.csv"
            df.to_csv(filename, index=False)
            rospy.loginfo(f"MPC: Performance data saved to {filename}")
        except Exception as e:
            rospy.logerr(f"MPC: Failed to save logs: {e}")

    def _record_state(self, tx, ty, tz, comp_time=0.0):
        """Record performance metrics."""
        try:
            (ax, ay, az), _ = self.gripper_pose
            err_dist = np.sqrt((tx - ax) ** 2 + (ty - ay) ** 2 + (tz - az) ** 2)
            data = {
                "time": rospy.Time.now().to_sec(),
                "target_x": tx,
                "target_y": ty,
                "target_z": tz,
                "actual_x": ax,
                "actual_y": ay,
                "actual_z": az,
                "error_dist": err_dist,
                "computation_time": comp_time,
            }
            for i, v in enumerate(self.current_joint_velocities):
                data[f"v{i+1}"] = v
            self.log_data.append(data)
        except Exception:
            pass


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

            self._record_state(tx, ty, tz, comp_time)
            rospy.sleep(1 / speed)

        if blocking:
            self.wait_for_position(tol_pos=0.005, tol_vel=0.08)

        self.gripper_pose = (x, y, z), target_quat

    def _record_state(self, tx, ty, tz, comp_time=0.0):
        if self.current_joint_state:
            try:
                ax, ay, az, _ = kinematics.get_pose(
                    self.current_joint_state.actual.positions
                )

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
