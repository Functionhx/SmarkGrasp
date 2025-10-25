#!/bin/bash
source ~/.bashrc
workspace=$(pwd)

#x7s
gnome-terminal -t "L" -x  bash -c "cd ${workspace}; cd ../..; cd ROS2/X5_ws; rm -rf build install log .catkin_workspace src/CMakeLists.txt;colcon build; exec bash;"
sleep 0.5

#VR
gnome-terminal -t "vr" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS2; rm -rf build install log .catkin_workspace;./port.sh; exec bash;"