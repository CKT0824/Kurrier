cmake_minimum_required(VERSION 3.0.2)
project(bbox_det)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vision_msgs
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
    FILES
    order_to_pwm.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    vision_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bbox_det src/bbox_det.cpp)
target_link_libraries(bbox_det 
  ${catkin_LIBRARIES}
) 

install(TARGETS bbox_det
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
