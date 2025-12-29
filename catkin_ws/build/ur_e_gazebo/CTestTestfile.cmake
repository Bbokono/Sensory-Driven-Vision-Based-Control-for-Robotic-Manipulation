# CMake generated Testfile for 
# Source directory: /root/catkin_ws/src/robot/universal_robot/ur_e_gazebo
# Build directory: /root/catkin_ws/build/ur_e_gazebo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_ur_e_gazebo_roslaunch-check_tests_roslaunch_test.xml "/root/catkin_ws/build/ur_e_gazebo/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/root/catkin_ws/build/ur_e_gazebo/test_results/ur_e_gazebo/roslaunch-check_tests_roslaunch_test.xml.xml" "--return-code" "/usr/bin/cmake -E make_directory /root/catkin_ws/build/ur_e_gazebo/test_results/ur_e_gazebo" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/root/catkin_ws/build/ur_e_gazebo/test_results/ur_e_gazebo/roslaunch-check_tests_roslaunch_test.xml.xml\" \"/root/catkin_ws/src/robot/universal_robot/ur_e_gazebo/tests/roslaunch_test.xml\" ")
set_tests_properties(_ctest_ur_e_gazebo_roslaunch-check_tests_roslaunch_test.xml PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/root/catkin_ws/src/robot/universal_robot/ur_e_gazebo/CMakeLists.txt;11;roslaunch_add_file_check;/root/catkin_ws/src/robot/universal_robot/ur_e_gazebo/CMakeLists.txt;0;")
subdirs("gtest")
