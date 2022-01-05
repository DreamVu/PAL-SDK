#!/bin/bash
cd installations 
./ros_cmake.sh
cp -r ../dreamvu_pal_camera ~/dev_ws/src/
cd ~/dev_ws/

source /opt/ros/foxy/setup.bash
colcon build --cmake-clean-cache 

. install/setup.bash
cp src/dreamvu_pal_camera/launch/pal_rviz.launch install/dreamvu_pal_camera/share/dreamvu_pal_camera/
cp src/dreamvu_pal_camera/launch/detection_rviz.launch install/dreamvu_pal_camera/share/dreamvu_pal_camera/
ros2 launch dreamvu_pal_camera pal_rviz.launch

