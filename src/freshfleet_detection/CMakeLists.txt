cmake_minimum_required(VERSION 3.0.2)
project(freshfleet_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
  trajectory_msgs
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    cv_bridge
    image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/object_detector.py
  scripts/robot_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 