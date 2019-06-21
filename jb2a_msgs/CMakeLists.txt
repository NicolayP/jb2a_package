cmake_minimum_required(VERSION 2.8.3)
project(jb2a_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
)

 add_action_files(
   FILES
   drive.action
 )

 generate_messages(
   DEPENDENCIES
   actionlib_msgs
 )


catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    message_runtime
    actionlib_msgs
  DEPENDS
)
