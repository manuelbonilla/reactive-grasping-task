cmake_minimum_required(VERSION 2.8.3)
project(reactive_grasping)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
  moveit_ros_planning_interface
  tf
)

add_message_files(
  DIRECTORY msg
  FILES
    GloveIMU.msg
    GloveIMUArray.msg
    DataHistory.msg
)

add_action_files(
  DIRECTORY action
  FILES
    Motion.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
    sensor_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)

set(BIN_DETECTION detection)
set(BIN_MOTION motion)

# Detection:
add_executable(${BIN_DETECTION}
  src/detection.cpp
  src/reactive_grasping_detection.cpp
)
target_link_libraries(${BIN_DETECTION}
  ${catkin_LIBRARIES}
)
add_dependencies(${BIN_DETECTION}
  ${catkin_EXPORTED_TARGETS}
)

# Motion:
add_executable(${BIN_MOTION}
  src/motion.cpp
  src/reactive_grasping_motion.cpp
)
target_link_libraries(${BIN_MOTION}
  ${catkin_LIBRARIES}
)
add_dependencies(${BIN_MOTION}
  ${catkin_EXPORTED_TARGETS}
)
