#!/usr/bin/python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion


def spawn_obstacle():
    rospy.init_node("spawn_obstacle")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawn_model_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    # Define a simple box obstacle SDF
    sdf_model = """
    <sdf version='1.6'>
      <model name='obstacle_box'>
        <static>true</static>
        <link name='link'>
          <collision name='collision'>
            <geometry>
              <box>
                <size>0.2 0.05 0.3</size>
              </box>
            </geometry>
          </collision>
          <visual name='visual'>
            <geometry>
              <box>
                <size>0.2 0.05 0.3</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Red</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """

    # Position: Between the robot home and the table center
    pose = Pose(Point(0.3, -0.2, 0.9), Quaternion(0, 0, 0, 1))
    spawn_model_client("obstacle_box", sdf_model, "", pose, "world")
    rospy.loginfo("Obstacle spawned successfully")


if __name__ == "__main__":
    spawn_obstacle()
