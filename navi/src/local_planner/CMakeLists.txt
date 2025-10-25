cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(Eigen3_DIR /usr/lib/cmake/eigen3)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  yhs_can_msgs
  Eigen3 REQUIRED
)

add_definitions(${EIGEN_DEFINITIONS})
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   robots_msgs#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_planner
#  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs nav_msgs robots_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(/usr/local/lib)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /usr/local/include/yaml-cpp
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/local_planner.cpp
# )

add_executable(local_planner src/local_planner.cpp)
add_dependencies(local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_planner ${catkin_LIBRARIES} )