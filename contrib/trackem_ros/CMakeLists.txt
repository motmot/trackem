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
install(PROGRAMS
  nodes/forward.py
  nodes/forwardcal.py
  nodes/calecho.py
  nodes/cloudecho.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  launch/pcloud.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)