cmake_minimum_required(VERSION 3.8)
project(arx_x5_controller)

find_package(catkin REQUIRED COMPONENTS
        arm_control
        arx5_arm_msg
        roscpp
        roslib
        std_msgs
        message_generation
        tf2
        tf2_ros
)

find_package(Eigen3 REQUIRED)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message(STATUS "Target architecture is ARM")
    set(DEPEND_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x5_src/libarx_x5_src-arm64.so)
else ()
    set(DEPEND_LIB ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_x5_src/libarx_x5_src.so)
endif ()

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS arm_control arx5_arm_msg roscpp std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

# 添加共享库路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)
add_executable(te src/te.cpp)
target_include_directories(te PUBLIC lib/arx_hardware_interface/include lib/arx_x5_src/include)
target_link_libraries(te ${catkin_LIBRARIES} ${DEPEND_LIB})
add_dependencies(te ${catkin_EXPORTED_TARGETS})
# 添加X5Controller可执行文件
add_executable(X5Controller src/X5Controller.cpp)
target_include_directories(X5Controller PUBLIC lib/arx_hardware_interface/include lib/arx_x5_src/include)
target_link_libraries(X5Controller ${catkin_LIBRARIES} ${DEPEND_LIB})
add_dependencies(X5Controller ${catkin_EXPORTED_TARGETS})

# 添加KeyBoard可执行文件
add_executable(KeyBoard src/KeyBoard.cpp)
target_include_directories(KeyBoard PUBLIC lib/arx_hardware_interface/include lib/arx_x5_src/include)
target_link_libraries(KeyBoard ${catkin_LIBRARIES} ${DEPEND_LIB})
add_dependencies(KeyBoard ${catkin_EXPORTED_TARGETS})

# 新增trajectory_generator可执行文件
add_executable(arx_control_node 
        src/trajectory_generator.cpp
        src/arx_arm_node.cpp
        src/arx_fsm.cpp)
target_include_directories(arx_control_node PUBLIC lib/arx_hardware_interface/include lib/arx_x5_src/include)
target_link_libraries(arx_control_node ${catkin_LIBRARIES})
add_dependencies(arx_control_node ${catkin_EXPORTED_TARGETS} )

# 安装目标文件
install(TARGETS
        X5Controller
        KeyBoard
        arx_control_node
        DESTINATION lib/${PROJECT_NAME})

install(FILES ${DEPEND_LIB}
        DESTINATION lib/
)