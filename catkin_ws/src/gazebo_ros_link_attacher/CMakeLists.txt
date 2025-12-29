cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_link_attacher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  message_generation
  std_msgs
)

find_package(gazebo REQUIRED)

add_service_files(
  FILES
  Attach.srv
  Detach.srv
  SetStatic.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_ros_link_attacher src/gazebo_ros_link_attacher.cpp)
target_link_libraries(gazebo_ros_link_attacher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(gazebo_ros_link_attacher ${PROJECT_NAME}_generate_messages_cpp)