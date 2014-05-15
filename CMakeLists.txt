cmake_minimum_required(VERSION 2.8.3)
project(pandora_rqt_gui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_mypkg
#  CATKIN_DEPENDS rospy rqt_gui rqt_gui_py
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


