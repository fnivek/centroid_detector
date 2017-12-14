cmake_minimum_required(VERSION 2.8.3)
project(centroid_detector_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  DetectCentroid.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
)