# Drone Swarm FYP

## Directory Structure

### src

src contains the python files which run the swarming logic.
- The 3D directory contains all of the improved code. This includes: a state machine, data logging, kalman filter and 3D multilateration.
- The 3D_controller directory is currently under development. This includes a central based controller to monitor and control the states of each drone using a GUI. Another GUI has also been developed called "ssh_gui.py". This allows the required scripts to be run without multiple terminals.
- The logs directory contains data logging for each practical flight test that was conducted.
- DATACOLLATION contains tools to visualise the data logging. 


### PX4-Autopilot

This contains firmware for running the PX4 flight controller and launching
Gazebo. The folder is forked from PX4 [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot),
and modified.

The launch folder contains launch files, for setting up Mavros communications and the Gazebo simultion.

`insect_tracking_mavros.launch` is the launch file for a flight test with a real PX4 controller.

`full_swarm_simulated.launch` launches Gazebo with five drones, each with an emulated PX4 controller.

`/Tools/sitl_gazebo/worlds/empty.world` is the world launched for simulation. The GPS location here can be
modified to change where the drones spwan e.g. to a location close to GPS data. An altitude can also be added to the world.

`/integrationtests/python_src/px4_it/mavros` contains example code written by PX4.

## Installation
### Requirements
- Ubuntu 20.04 LTS
- Python3.8.X

## Instructions
1. Once the repository is cloned pull the PX4-Autopilot submodule to the cloned repository using:<br/>
`cd drone-swarm-fyp`<br/>
`git submodule update --init --recursive`<br/>
2. Next install the required packages including ROS Noetic and Mavros by running the shell script "ubuntu_sim_ros_noetic.sh":<br/>
`bash ubuntu_sim_ros_noetic.sh`<br/>
3. Restart the PC using:<br/>
`sudo reboot now`<br/>
4. Once rebooted make the simulation using:
`cd PX4-Autopilot`<br/>
`DONT_RUN=1 make px4_sitl_default gazebo`<br/>
5. Download QGroundControl from: [http://qgroundcontrol.com/downloads/](http://qgroundcontrol.com/downloads/) and update the file's permissions by running:<br/>
`cd ~/Downloads`<br/>
`chmod +x ./QGroundControl.AppImage`<br/>
6. To run QGroundControl double click it or type:<br/>
`./QGroundControl.AppImage`<br/>
7. Once this is complete test the setup by running the five drone simulation from the PX4-Autopilot folder:<br/>
`source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`<br/>
`roslaunch px4 full_swarm_simulated.launch`<br/>
8. Gazebo should open and the drones should appear in QGroundControl. If it say that the drones are not ready and need calibrated delete the sitl_iris_0 to sitl_iris_5 in the .ros directory in your home folder. It is hidded by default in the file explorer so use ctrl-h to show hidden files. If this occours restart from step 7.<br/>
9. To conect to the drones navigate to ../drone-swarm-fyp/src/LastYear in a new terminal and run:<br/>
`python3 run_drones.py -a`<br/>
-a means autoarm. Optionally the -p flag may be added to plot the path of the drones. Push enter 5 times (one for each drone) to move them to offboard mode. <br/>
10. Next run:<br/>
`python3 -m server_data`<br/>
This script broadcasts pre-recorded GPS positions to the drones for them to follow. The drones should start moving in QGroundControl and Gazebo.<br/>

## Flight Tests
### Simulation
The simulation can be run with a varying amount of drones 