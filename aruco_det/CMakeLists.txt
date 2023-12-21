cmake_minimum_required(VERSION 3.0.2)
project(aruco_det)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  image_transport
  std_msgs
  message_generation  # Add this line for message generation
)

# Declare the message files to be built
add_message_files(
  FILES
  order_to_pwm_marker.msg
)

# Generate the message
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_det
  CATKIN_DEPENDS std_msgs cv_bridge roscpp image_transport message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(aruco_det src/aruco_det.cpp)

target_link_libraries(aruco_det
  ${catkin_LIBRARIES}
)