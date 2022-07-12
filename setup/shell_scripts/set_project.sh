#!/bin/bash
#
# Copyright (c) 2021 Wei-Chih Lin(weichih.lin@protonmail.com)
#
# This file will help you setup the environment for ROSKY2 
# Support platform:
#   1. reComputer J1010 with Jetpack 4.6 
#      (https://www.icshop.com.tw/product-page.php?28703)
#   2. Jetson Nano Developer kit for third party Ubuntu 20.04 with Jetpack 4.3
#      (https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768)
#



#######################################
# Setup parameter for project ROSKY2
# Globals:
#   None
# Arguments:
#   None
#######################################
setup_parameter(){
    echo "No parameter need to set." 
}

#######################################
# Build Project
# Globals:
#   None
# Arguments:
#   project name
#######################################
colcon_build(){
    cd ${HOME}/${1}/ros2_ws && colcon build --symlink-install
}

#######################################
# Insert commands in ros_menu/config.yaml
# Globals:
#   None
# Arguments:
#   project name
#######################################
insert_cmds(){
    if [ -z "$(cat ${HOME}/ros_menu/config.yaml | grep ${1})" ] 
    then
        sed -i "24 a \ \ \ \ \ \ -\ source\ ${HOME}/${1}/setup/shell_scripts/environment.sh" ${HOME}/ros_menu/config.yaml
    fi

}


#######################################
# Main function
# Globals:
#   None
# Arguments:
#   None
#######################################
main(){
    echo -e "Start setting the project ROSKY2..."
    setup_parameter
    colcon_build ROSKY2
    insert_cmds ROSKY2
    echo -e "Done! Please re-source the file ${HOME}/.bashrc"
}

# start the progress
main




