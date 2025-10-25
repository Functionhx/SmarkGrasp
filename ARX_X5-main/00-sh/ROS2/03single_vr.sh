#!/bin/bash

workspace=$(pwd)
source ~/.bashrc

# CAN
gnome-terminal -t "can" -x sudo bash -c "cd ${workspace};cd ../.. ; cd ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 1
#x7s
gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../..; cd ROS2/X5_ws; source install/setup.bash && ros2 launch arx_x5_controller open_vr_single_arm.launch.py; exec bash;"
sleep 0.1
#VR
gnome-terminal -t "unity_tcp" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS2; source install/setup.bash;ros2 run serial_port serial_port_node;exec bash;"
sleep 0.1
gnome-terminal -t "arx5_pos_cmd" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS2; source install/setup.bash;ros2 topic echo /ARX_VR_L;exec bash;"
