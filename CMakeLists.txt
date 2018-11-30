cmake_minimum_required(VERSION 2.8.3)
project(tof_ble_drone)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  rospy
)

add_message_files(
  FILES  HandleDroneData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

