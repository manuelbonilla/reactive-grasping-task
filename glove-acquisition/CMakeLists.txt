cmake_minimum_required(VERSION 2.8.3)
project(glove_acquisition)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  cmake_modules #need for eigen
)

add_message_files(
  DIRECTORY msg
  FILES
    GloveIMU.msg
    GloveIMUArray.msg
)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(LIB_GLOVE glove_core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIB_GLOVE}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)

set(BIN_TALKER glove_talker)
set(BIN_LISTENER glove_listener)

# Talker
add_executable(${BIN_TALKER}
  src/glove_talker.cpp
  src/glove_core.cpp
)
target_link_libraries(${BIN_TALKER} 
  ${catkin_LIBRARIES}
)
add_dependencies(${BIN_TALKER} 
  ${catkin_EXPORTED_TARGETS}
)

# Listener
add_executable(${BIN_LISTENER}
  src/glove_listener.cpp
  src/glove_core.cpp
)
target_link_libraries(${BIN_LISTENER}
  ${catkin_LIBRARIES}
)
add_dependencies(${BIN_LISTENER} 
  ${catkin_EXPORTED_TARGETS}
)

# Exported library
add_library(${LIB_GLOVE}
  src/glove_core.cpp
)
target_link_libraries(${LIB_GLOVE}
  ${catkin_LIBRARIES}
)
add_dependencies(${LIB_GLOVE} 
  ${catkin_EXPORTED_TARGETS}
)
