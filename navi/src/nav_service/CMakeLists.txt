cmake_minimum_required(VERSION 3.0.2)
project(nav_service)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arm_control
  arx5_arm_msg
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  yhs_can_msgs
  actionlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES nav_service
#  CATKIN_DEPENDS arm_control arx5_arm_msg geometry_msgs roscpp rospy sensor_msgs std_msgs yhs_can_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/infer_navigator.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)