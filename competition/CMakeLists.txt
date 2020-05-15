cmake_minimum_required(VERSION 2.8.3)
project(competition)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  tf
)

catkin_python_setup()

generate_messages (
    DEPENDENCIES std_msgs geometry_msgs tf
)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs tf)

catkin_install_python(
    PROGRAMS
        nodes/line_follower.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

