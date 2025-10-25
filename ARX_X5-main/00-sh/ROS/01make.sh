#!/bin/bash
source ~/.bashrc
workspace=$(pwd)

#x7s
gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../..; cd ROS/X5_ws; rm -rf build devel .catkin_workspace src/CMakeLists.txt; catkin_make clean ;catkin_make; exec bash;"
sleep 0.5

#VR
gnome-terminal -t "vr" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS; rm -rf build devel .catkin_workspace;./port.sh; exec bash;"