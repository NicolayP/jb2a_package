cmake_minimum_required(VERSION 2.8.3)
project(jb2a_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  jb2a_msgs
  actionlib
  )
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    jb2a_msgs
  DEPENDS
    Boost
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(drive_server src/drive_server.cpp)

target_link_libraries(
  drive_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  drive_server
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

#
#add_executable(simpleAction_client src/simpleAction_client.cpp)
#
#target_link_libraries(
#  simpleAction_client
#  ${catkin_LIBRARIES}
#)

#add_dependencies(
#  simpleAction_client
#  ${PROJECT_NAME}
#  ${catkin_EXPORTED_TARGETS}
#)
