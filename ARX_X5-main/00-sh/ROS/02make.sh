#!/bin/bash
workspace=$(pwd)

source ~/.bashrc

#VR
gnome-terminal -t "vr" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS; rm -rf build devel .catkin_workspace;catkin_make; exec bash;"