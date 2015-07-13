cmake_minimum_required(VERSION 2.8.3)
project(reactive_grasping_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY robot DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})