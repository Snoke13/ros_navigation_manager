cmake_minimum_required(VERSION 2.8.3)
project(pepper_pose_for_nav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs tf dynamic_reconfigure)


 #Generate services in the 'srv' folder
 add_service_files(
   FILES
   MoveHeadAtPosition.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
   INCLUDE_DIRS
#  LIBRARIES laser_merger
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)
