cmake_minimum_required(VERSION 2.8.3)
project(yhs_can_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  yhs_can_msgs
)

#find_package(yhs_can_msgs REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_node
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

link_directories(
　 include
  include/yhs_can_control
  ${catkin_INCLUDE_DIRS} yhs_can_control
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 　include
  include/yhs_can_control
  ${catkin_INCLUDE_DIRS} yhs_can_control
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tcpclient.cpp
# )


#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/yhs_can_control.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)


## 灯控节点
add_executable(lamp_node src/lamp_node.cpp)
add_dependencies(lamp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lamp_node ${catkin_LIBRARIES})

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}_node lamp_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)