cmake_minimum_required(VERSION 2.8.3)
project(trackem_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  MTPoint.msg
  MTPoints.msg
  MTCalPoint.msg
  MTCalPoints.msg
 )
generate_messages()

catkin_package()
install()