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
echo "Please read license agreement for each component and continue only if you accept the license terms."
echo "ROS Kinetic  : http://www.ros.org/"
echo "MAVROS       : http://github.com/mavlink/mavros"
echo "raspicam_node: https://github.com/UbiquityRobotics/raspicam_node"
echo "${reset}"