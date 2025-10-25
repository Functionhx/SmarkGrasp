#!/bin/bash
workspace=$(pwd)

source ~/.bashrc

#VR
gnome-terminal -t "vr" -x  bash -c "cd ${workspace}; cd ../..; cd ARX_VR_SDK/ROS2; rm -rf build install log .catkin_workspace src/CMakeLists.txt;colcon build; exec bash;"