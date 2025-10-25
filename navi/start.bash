# !/bin/bash
sudo /usr/local/bin/setup_can.sh

source /home/yh/livox_ws/devel/setup.bash
gnome-terminal -- bash -c "roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"
sleep 3s

source /home/yh/point_lio/devel/setup.bash 
gnome-terminal -- bash -c "roslaunch point_lio mapping_mid360.launch; exec bash"

source /home/yh/navi/devel/setup.bash

sleep 1.5s  
gnome-terminal -- bash -c "roslaunch depth_cluster_ros depth_cluster_ros.launch; exec bash"
sleep 1.5s 
gnome-terminal -- bash -c "roslaunch local_planner navigation.launch; exec bash"
# gnome-terminal -- bash -c "roslaunch local_planner slam.launch; exec bash"
sleep 1.5s 

source /home/yh/FW-mid-ros1/devel/setup.bash
gnome-terminal -- bash -c "roslaunch yhs_can_control yhs_can_control.launch; exec bash"
# gnome-terminal -- bash -c "roslaunch pclfilter clear.launch; exec bash"
# sleep 1.5s 