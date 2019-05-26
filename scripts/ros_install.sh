#!/bin/bash

green=`tput setaf 2`
red=`tput setaf 1`
reset=`tput sgr0`

# Should not run this script as sudo.
if [ "$EUID" = 0 ]; then
    echo "${red}Please run this script as a non-root user.${reset}"
    exit
fi

echo "${green}This script will install several components."
echo "The components have diferents licenses check each one before using this repo."
echo "ROS Kinetic  : http://www.ros.org/"
echo "MAVROS       : http://github.com/mavlink/mavros"
echo "raspicam_node: https://github.com/UbiquityRobotics/raspicam_node"
echo "${reset}"

# We are using the UbiquityRobotics ubuntu images so we not need to install ros

# Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116

# Update package index
sudo apt-get update

# Install MAVROS packages
sudo apt-get install -y ros-kinetic-mavros ros-kinetic-mavros-extras

# MAVROS requires GeographicLib datasets starting v0.20 .
sudo geographiclib-get-geoids egm96-5

# Initialize rosdep
sudo rosdep init
rosdep update

# Set the custom catkin workspace
CCWS=$HOME/catkin_custom

# Check if CCWS already exist 
if [ ! -d "$CCWS" ]; then
    echo "${green}Creating catkin workspace in $CCWS...${reset}"
    mkdir -p $CCWS/src
    cd $CCWS/src
    catkin_init_workspace
fi

# Installing raspberry cam node from UbiquityRobotics

echo "${green}Starting installation of raspicam_node ROS package...${reset}"
echo "Adding ubiquity robotics ros source to rosdep"

sudo sh -c 'echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" > /etc/ros/rosdep/sources.list.d/30-ubiquity.list'

rosdep update

cd $HOME

# Install raspicam_node from sources rather than apt-get install as the latter installs a lot of redundant stuff.

if [ ! -d "$HOME/raspicam_node" ]; then
    echo "Cloning raspicam_node sources..."
    git clone https://github.com/pedrogasg/raspicam_node.git
    cd raspicam_node
    git checkout tencendur
    # Create symlink to catkin workspace.
    ln -s $HOME/raspicam_node $CCWS/src/
else
    echo "Updating raspicam_node sources..."
    cd raspicam_node
    git pull
fi

echo "Building raspicam_node package..."
cd $CCWS
catkin_make