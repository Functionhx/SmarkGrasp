# cmake_minimum_required(VERSION 3.10)
# project(pclfilter)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   sensor_msgs
#   pcl_ros
# )

# find_package(PCL 1.8 REQUIRED COMPONENTS
#   common
#   segmentation
#   features
#   filters
#   io
#   visualization
# )

# find_package(Eigen3 REQUIRED)

# catkin_package(
#   CATKIN_DEPENDS
#   roscpp
#   std_msgs
#   sensor_msgs
#   pcl_ros
#   tf2_sensor_msgs
# )

# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
# )

# link_directories(${PCL_LIBRARY_DIRS})
# # clear
# add_executable(clear
#   src/use_clear.cpp
#   include/clear.cpp
# )

# target_link_libraries(clear
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   Eigen3::Eigen
# )

# add_dependencies(clear ${catkin_EXPORTED_TARGETS})

# # depth_cluster
# add_library(depth_cluster include/depth_cluster.cpp)
# target_link_libraries(depth_cluster ${PCL_LIBRARIES})

# add_executable(depth_cluster_node src/main.cpp)
# target_link_libraries(depth_cluster_node depth_cluster ${catkin_LIBRARIES})


# # # depth_cluster_test

# # add_executable(depth_cluster_test src/test.cpp)
# # target_link_libraries(depth_cluster_test depth_cluster ${catkin_LIBRARIES})
