cmake_minimum_required(VERSION 2.8.3)
project(aiibot_bringup)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
  diff_drive_controller
  geometry_msgs
  robot_localization
  rosconsole
  roscpp
  sensor_msgs
  tf
  std_srvs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)
target_link_libraries(tf_broadcaster_node
  ${catkin_LIBRARIES}
)

add_executable(monitor_serial_node src/monitor_serial_node.cpp)
target_link_libraries(monitor_serial_node
  ${catkin_LIBRARIES}
)

add_executable(cmd_boost_node src/cmd_boost_node.cpp)
target_link_libraries(cmd_boost_node
  ${catkin_LIBRARIES}
)

