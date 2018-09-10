#!/bin/bash

green=`tput setaf 2`
red=`tput setaf 1`
reset=`tput sgr0`

# Disable and erase roscore if exist
if [ ! -f /etc/systemd/system/roscore.service ]; then
    echo "${red}Disabling roscore and remplacing the file.${reset}"
    sudo systemctl disable roscore.service
    sudo rm -f /etc/systemd/system/roscore.service
fi

# Adding roscore
sudo systemctl enable $HOME/tencendur/services/roscore.service

# Adding mavros
sudo systemctl enable $HOME/tencendur/services/mavros.service