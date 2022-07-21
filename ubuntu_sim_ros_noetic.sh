#!/bin/bash

## Bash script for setting up ROS Noetic (with Gazebo 11) development environment for PX4 on Ubuntu LTS (20.04). 
## It installs the common dependencies for all targets (including Qt Creator)
##
## Installs:
## - Common dependencies libraries and tools as defined in `ubuntu_sim_common_deps.sh`
## - ROS Noetic (including Gazebo11)
## - MAVROS

if [[ $(lsb_release -sc) == *"focal"* ]]; then
  echo "OS version detected as $(lsb_release -sc) (20.04)."
  echo "ROS Noetic requires at least Ubuntu 20.04."
  echo "Exiting ...."
fi

# Preventing sudo timeout https://serverfault.com/a/833888
trap "exit" INT TERM; trap "kill 0" EXIT; sudo -v || exit $?; sleep 1; while true; do sleep 60; sudo -nv; done 2>/dev/null &

# Ubuntu Config
echo "Remove modemmanager"
sudo apt-get remove modemmanager -y
echo "Add user to dialout group for serial port access (reboot required)"
sudo usermod -a -G dialout $USER

# Common dependencies
echo "Installing common dependencies"
sudo apt-get update -y
sudo apt-get install git zip cmake build-essential genromfs ninja-build exiftool astyle -y
# make sure xxd is installed, dedicated xxd package since Ubuntu 18.04 but was squashed into vim-common before
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y
# Required python packages
sudo apt-get install python-argparse python3-empy python3-toml python3-numpy python3-dev python3-pip -y
sudo -H pip3 install --upgrade pip
sudo -H pip3 install pandas jinja2 pyserial pyyaml
# optional python tools
sudo -H pip3 install pyulog

# Ubuntu 20.04 extra by ekaktusz
sudo apt-get install python3-osrf-pycommon python3-catkin-tools
sudo apt-get install ros-noetic-geographic-msgs
sudo apt-get install python3-pip

pip3 install future
pip3 install osrf-pycommon
pip3 install lxml
pip3 install scipy
pip3 install filterpy
pip3 install pymavlink
pip3 install json5
pip3 install zmq
pip3 install jsonschema
pip3 install packaging
pip3 install pyros-genmsg

sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev


# ROS Noetic
## Gazebo simulator dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

## ROS Gazebo: http://wiki.ros.org/noetic/Installation/Ubuntu
## Setup keys
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
## For keyserver connection problems substitute hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 above.
sudo apt-get update
## Get ROS/Gazebo
sudo apt install ros-noetic-desktop-full python3-rosdep -y
## Initialize rosdep
sudo rosdep init
rosdep update
## Setup environment variables
rossource="source /opt/ros/noetic/setup.bash"
if grep -Fxq "$rossource" ~/.bashrc; then echo ROS setup.bash already in .bashrc;
else echo "$rossource" >> ~/.bashrc; fi
eval $rossource

## Install rosinstall and other dependencies
sudo apt install python3-rosinstall python3-rosinstall-generator python3-roslaunch python3-wstool build-essential -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y


# MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
## Install dependencies
sudo apt-get install python3-catkin-tools python3-rosinstall-generator -y

## Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src


## Install MAVLink
###we use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

## Build MAVROS
### Get source (upstream - released)
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

### Setup workspace & install deps
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
if ! rosdep install --from-paths src --ignore-src -y; then
    # (Use echo to trim leading/trailing whitespaces from the unsupported OS name
    unsupported_os=$(echo $(rosdep db 2>&1| grep Unsupported | awk -F: '{print $2}'))
    rosdep install --from-paths src --ignore-src --rosdistro noetic -y --os ubuntu:bionic
fi

if [[ ! -z $unsupported_os ]]; then
    >&2 echo -e "\033[31mYour OS ($unsupported_os) is unsupported. Assumed an Ubuntu 18.04 installation,"
    >&2 echo -e "and continued with the installation, but if things are not working as"
    >&2 echo -e "expected you have been warned."
fi

#Install geographiclib
sudo apt install geographiclib-tools -y
echo "Downloading dependent script 'install_geographiclib_datasets.sh'"
# Source the install_geographiclib_datasets.sh script directly from github
install_geo=$(wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh -O -)
wget_return_code=$?
# If there was an error downloading the dependent script, we must warn the user and exit at this point.
if [[ $wget_return_code -ne 0 ]]; then echo "Error downloading 'install_geographiclib_datasets.sh'. Sorry but I cannot proceed further :("; exit 1; fi
# Otherwise source the downloaded script.
sudo bash -c "$install_geo"

## Build!
catkin build
## Re-source environment to reflect new packages/build environment
catkin_ws_source="source ~/catkin_ws/devel/setup.bash"
if grep -Fxq "$catkin_ws_source" ~/.bashrc; then echo ROS catkin_ws setup.bash already in .bashrc; 
else echo "$catkin_ws_source" >> ~/.bashrc; fi
eval $catkin_ws_source
