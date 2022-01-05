#!/bin/bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-foxy-desktop
sudo apt install ros-foxy-ros-base
sudo apt install python3-colcon-common-extensions   
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
source /opt/ros/foxy/setup.bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
ros2 pkg create --build-type ament_cmake dreamvu_pal_camera
