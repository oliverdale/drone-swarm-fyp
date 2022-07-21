#!/bin/bash

# Used by sshgui.py, used to launch the gazebo simulation from a python process.
# Sets the required shell environment variables and launches a simulation 
# with the required number of drones. 
# Takes an argument specifing the number of simulated receiver drones to launch.

# Author: Oliver Dale
# Date: 25/09/21

cd ../..;
source /opt/ros/noetic/setup.bash; 
source ~/catkin_ws/devel/setup.bash; 
cd PX4-Autopilot; 
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default; 

if [ "$1" == "1234" ]; then
	roslaunch px4 full_swarm_simulated.launch
else
	roslaunch px4 uav_0$1.launch
fi




