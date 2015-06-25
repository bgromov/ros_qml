cmake_minimum_required(VERSION 2.8.3)
project(ros_qml)

find_package(catkin REQUIRED COMPONENTS
  roslib
  rospy
)

catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/ros_qml
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    
install(DIRECTORY plugins
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})