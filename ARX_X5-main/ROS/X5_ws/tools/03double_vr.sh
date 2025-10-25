#!/bin/bash
source ~/.bashrc
workspace=$(pwd)

# CAN
gnome-terminal -t "can1" -x bash -c "cd ${workspace}/ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
# R5
gnome-terminal -t "R5" -x  bash -c "cd ${workspace};cd ROS2/R5_ws; source install/setup.bash && ros2 launch arx_r5_controller open_vr_double_arm.launch.py; exec bash;"

# VR
gnome-terminal -t "unity_tcp" -x bash -c "cd ${workspace};cd ..;cd ARX_VR_SDK/ROS2 ;source install/setup.bash && ros2 run serial_port serial_port_node;exec bash;"
sleep 1
gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace};cd ..;cd ARX_VR_SDK/ROS2 ;source install/setup.bash && ros2 topic echo /ARX_VR_L;exec bash;"
roslaunch realsense2_camera rs_camera.launch
