cmake_minimum_required(VERSION 2.8.3)
project(centroid_detector_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  actionlib_msgs
  sensor_msgs
)

# Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
    DetectCentroid.action
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  actionlib_msgs
  sensor_msgs
)