cmake_minimum_required(VERSION 2.8.3)
project(TiagoBears_ColorDetection)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_ros
  control_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  StringArray.msg
)

add_service_files(
  FILES
  Getcolor.srv
  InitEmpty.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  
  CATKIN_DEPENDS message_runtime
  rospy
  std_msgs
)

catkin_install_python(PROGRAMS scripts/color_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




