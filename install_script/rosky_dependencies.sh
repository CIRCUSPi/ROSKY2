#!/bin/bash
#
#Copyright (c) 2021 Wei-Chih Lin(weichih.lin@protonmail.com)
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#

# Python dependencies
ubuntu_distro=$(grep RELEASE /etc/lsb-release | awk -F '=' '{print $2}')
ros_distro=foxy

# install ROS2 dependencies
sudo apt install -y ros-$ros_distro-rqt-reconfigure


# install python3 dependencies through apt
sudo apt install -y python3-serial 


# install python3 dependencies
sudo -H pip3 install ruamel.yaml \
                     numpy \
                     
# config project ROSKY2 
source ${HOME}/ROSKY2/setup/python_scripts/config.py
