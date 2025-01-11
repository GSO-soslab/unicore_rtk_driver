cmake_minimum_required(VERSION 3.0.2)
project(unicore_rtk_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf 
  tf2_ros
  nmea_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES unicore_rtk_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
)

add_executable(crc_test test/crc_test.cpp)
target_link_libraries(crc_test ${thirdparty_libraries})
