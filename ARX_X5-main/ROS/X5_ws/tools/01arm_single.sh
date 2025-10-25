#!/bin/bash
source ~/.bashrc
workspace=$(pwd)
# /home/arx/桌面/sdk/R5_SDK_KDL/ROS/R5_ws/tools/01arm_single.sh
# CAN
gnome-terminal -t "can1" -x bash -c "cd ${workspace}/../../..;cd ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 1s
# R5
gnome-terminal -t "R5" -x  bash -c "cd ${workspace}/..; source devel/setup.bash && roslaunch arx_r5_controller open_single_arm.launch; exec bash;"

# gnome-terminal -t "R5" -x  bash -c "cd ${workspace}/..; source devel/setup.bash && rqt; exec bash;"
# gnome-terminal -t "R5" -x  bash -c "cd ${workspace}/..; source devel/setup.bash && roslaunch arx_r5_controller open_keyboard_control.launch; exec bash;"

# gnome-terminal -t "R5" -x  bash -c "cd ${workspace}/..; source devel/setup.bash && rosrun arx_r5_controller R5KeyBoard; exec bash;"
