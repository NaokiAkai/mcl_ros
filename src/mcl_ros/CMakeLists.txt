cmake_minimum_required(VERSION 3.0.2)
project(mcl_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mcl_ros
#  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
)

add_executable(mcl src/mcl.cpp)
target_link_libraries(mcl ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

