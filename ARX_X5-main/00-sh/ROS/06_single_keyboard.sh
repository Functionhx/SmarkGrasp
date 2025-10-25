#!/bin/bash

workspace=$(pwd)
source ~/.bashrc

# CAN
gnome-terminal -t "can" -x sudo bash -c "cd ${workspace};cd ../.. ; cd ARX_CAN/arx_can; ./arx_can1.sh; exec bash;"
sleep 1
#x7s
gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../..; cd ROS/X5_ws; source devel/setup.bash && roslaunch arx_x5_controller open_keyboard_control.launch; exec bash;"
sleep 3
gnome-terminal -t "keyboard" -x  bash -c "cd ${workspace}; cd ../..; cd ROS/X5_ws; source devel/setup.bash && rosrun arx_x5_controller KeyBoard; exec bash;"
