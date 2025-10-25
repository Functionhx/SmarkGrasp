#!/bin/bash
source ~/.bashrc
workspace=$(pwd)

# /home/arx/桌面/sdk/R5_SDK_KDL/ROS/R5_ws/tools/02single_vr.sh

gnome-terminal -t "can1" -x bash -c "cd ${workspace}/../../..;cd ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 1s
# R5
gnome-terminal -t "R5" -x  bash -c "cd ${workspace}/..; source devel/setup.bash && roslaunch arx_r5_controller open_vr_single_arm.launch; exec bash;"


# VR
gnome-terminal -t "unity_tcp" -x bash -c "cd ${workspace}/../../../..;cd ARX_VR_SDK/ROS ;source devel/setup.bash && rosrun serial_port serial_port;exec bash;"
sleep 1
gnome-terminal -t "arx5_pos_cmd" -x bash -c "cd ${workspace}/../../../..;cd ARX_VR_SDK/ROS;source devel/setup.bash && rostopic echo /ARX_VR_L;exec bash;"

