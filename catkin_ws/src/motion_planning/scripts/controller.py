import math
import copy
import rospy
import numpy as np
import kinematics
import control_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
from pyquaternion import Quaternion
import sys
import moveit_commander
import moveit_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
from scipy.optimize import minimize

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

        # Create an action client for the joint trajectory
        self.joints_pub = rospy.Publisher(
            f"{self.controller_topic}/command",
            trajectory_msgs.msg.JointTrajectory,
            queue_size=10,
        )

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
            self.send_joints(
                sx + i_1 * dx,
                sy + i_1 * dy,
                sz + i_1 * dz + i_2 * z_raise,
                grip,
                duration=1 / speed * 0.9,
            )
            rospy.sleep(1 / speed)

        if blocking:
            self.wait_for_position(tol_pos=0.005, tol_vel=0.08)

        self.gripper_pose = (x, y, z), target_quat

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


class MoveItArmController(ArmController):
    """
    Alternative controller that uses MoveIt for path planning (RRT, PRM, etc.)
    instead of straight-line interpolation.
    """

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("Initializing MoveIt! Controller...")
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"  # Standard UR5 group name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Set planner parameters
        self.move_group.set_planning_time(5.0)
        self.move_group.set_max_velocity_scaling_factor(0.5)
        self.move_group.set_max_acceleration_scaling_factor(0.5)

        # Disable self-collisions for the gripper
        self.disable_gripper_collisions()

        # Initialize parent for gripper state tracking
        super().__init__()

    def disable_gripper_collisions(self):
        """
        Disables collisions between all links of the Robotiq gripper in the Allowed Collision Matrix.
        This prevents START_STATE_IN_COLLISION errors due to the gripper's internal geometry.
        """
        pub = rospy.Publisher(
            "/planning_scene", moveit_msgs.msg.PlanningScene, queue_size=10, latch=True
        )

        # Wait for subscriber (MoveGroup) to connect
        rospy.loginfo("Waiting for /planning_scene subscriber...")
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("/planning_scene subscriber connected.")

        scene = moveit_msgs.msg.PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        # Identify gripper links
        gripper_links = [
            link for link in self.robot.get_link_names() if "robotiq_85" in link
        ]

        # Disable collisions between all gripper links
        acm = moveit_msgs.msg.AllowedCollisionMatrix()
        acm.entry_names = gripper_links
        for _ in gripper_links:
            entry = moveit_msgs.msg.AllowedCollisionEntry()
            entry.enabled = [False] * len(
                gripper_links
            )  # Disable collision against all other gripper links
            acm.entry_values.append(entry)

        scene.allowed_collision_matrix = acm
        pub.publish(scene)
        rospy.loginfo("Disabled gripper self-collisions in Planning Scene")

    def add_obstacle(self):
        """Adds a static obstacle to the MoveIt planning scene matching spawn_obstacle.py"""
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.x = 0.3
        box_pose.pose.position.y = -0.2
        box_pose.pose.position.z = 0.9
        box_pose.pose.orientation.w = 1.0
        self.scene.add_box("obstacle_box", box_pose, size=(0.2, 0.05, 0.3))
        rospy.loginfo("Obstacle added to MoveIt Planning Scene")

    def move_to(
        self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True
    ):
        (sx, sy, sz), start_quat = self.gripper_pose

        if x is None:
            x = sx
        if y is None:
            y = sy
        if z is None:
            z = sz
        if target_quat is None:
            target_quat = start_quat

        # Define target pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = target_quat.w
        pose_goal.orientation.x = target_quat.x
        pose_goal.orientation.y = target_quat.y
        pose_goal.orientation.z = target_quat.z
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        self.move_group.set_pose_target(pose_goal)

        # Plan and execute (MoveIt handles obstacle avoidance here)
        plan = self.move_group.go(wait=blocking)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        # Update internal state
        self.gripper_pose = (x, y, z), target_quat


class VelocityArmController(ArmController):
    """
    Controller for MPC / Visual Servoing.
    Publishes velocity commands instead of trajectories.
    """

    def __init__(self):
        # Initialize directly, skipping ArmController.__init__ to avoid waiting for trajectory controller
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.vel_pub = rospy.Publisher(
            "/joint_group_vel_controller/command",
            std_msgs.msg.Float64MultiArray,
            queue_size=1,
        )

        # Initialize pose tracking
        self.gripper_pose = ((0, 0, 0), Quaternion())
        joints = self.get_current_joints()
        if joints is not None:
            x, y, z, rot = kinematics.get_pose(joints)
            self.gripper_pose = ((x, y, z), Quaternion(matrix=rot))
        else:
            rospy.logwarn("VelocityArmController: Could not get initial joint state")

    def get_current_joints(self):
        try:
            msg = rospy.wait_for_message(
                "/joint_states", sensor_msgs.msg.JointState, timeout=1.0
            )
            if not msg:
                return None

            pos = []
            # Create a dict for faster lookup
            joint_map = dict(zip(msg.name, msg.position))

            for name in self.joint_names:
                if name in joint_map:
                    pos.append(joint_map[name])
                else:
                    return None
            return np.array(pos)
        except Exception:
            return None

    def send_velocity(self, velocities):
        """
        Send joint velocities.
        :param velocities: list of 6 floats [rad/s]
        """
        msg = std_msgs.msg.Float64MultiArray()
        msg.data = velocities
        self.vel_pub.publish(msg)

    def stop(self):
        self.send_velocity([0.0] * 6)

    def move_to(
        self, x=None, y=None, z=None, target_quat=None, z_raise=0.0, blocking=True
    ):
        # Get current pose to fill in None values
        (cx, cy, cz), c_quat = self.gripper_pose

        # Update current pose from sensors if possible
        joints = self.get_current_joints()
        if joints is not None:
            cx, cy, cz, rot = kinematics.get_pose(joints)
            c_quat = Quaternion(matrix=rot)
            self.gripper_pose = ((cx, cy, cz), c_quat)

        if x is None:
            x = cx
        if y is None:
            y = cy
        if z is None:
            z = cz
        if target_quat is None:
            target_quat = c_quat

        # Execute control loop
        self.control_loop(x, y, z, target_quat)

        # Update internal state
        self.gripper_pose = ((x, y, z), target_quat)

    def control_loop(self, tx, ty, tz, t_quat):
        rate = rospy.Rate(50)

        # Timeout to prevent infinite loops
        start_time = rospy.Time.now()
        timeout = rospy.Duration(10.0)

        while not rospy.is_shutdown():
            if rospy.Time.now() - start_time > timeout:
                rospy.logwarn("MPC Timeout")
                self.stop()
                break

            q_curr = self.get_current_joints()
            if q_curr is None:
                continue

            # Forward Kinematics
            cx, cy, cz, crot = kinematics.get_pose(q_curr)
            c_quat = Quaternion(matrix=crot)

            # Error calculation
            pos_error = np.array([tx - cx, ty - cy, tz - cz])
            dist_error = np.linalg.norm(pos_error)

            # Orientation error: 1 - |q1.dot(q2)|
            orient_error = 1.0 - abs(np.dot(c_quat.elements, t_quat.elements))

            # Convergence check
            if dist_error < 0.005 and orient_error < 0.01:
                self.stop()
                break

            # MPC Optimization
            dt = 0.02  # 50Hz prediction horizon

            def objective(q_dot):
                q_next = q_curr + q_dot * dt
                nx, ny, nz, nrot = kinematics.get_pose(q_next)
                n_quat = Quaternion(matrix=nrot)

                p_err = (nx - tx) ** 2 + (ny - ty) ** 2 + (nz - tz) ** 2
                o_err = 1.0 - abs(np.dot(n_quat.elements, t_quat.elements))
                reg = np.sum(q_dot**2)  # Regularization

                return 100.0 * p_err + 10.0 * o_err + 0.001 * reg

            # Bounds for UR5 velocity (rad/s)
            bounds = [(-1.0, 1.0) for _ in range(6)]
            x0 = np.zeros(6)

            # SLSQP is suitable for constrained optimization
            res = minimize(
                objective,
                x0,
                method="SLSQP",
                bounds=bounds,
                options={"ftol": 1e-3, "disp": False},
            )

            if res.success:
                self.send_velocity(res.x)
            else:
                self.send_velocity(np.zeros(6))

            rate.sleep()
