cd installations 
./ros_cmake.sh
cp -r ../dreamvu_pal_camera/ ~/catkin_ws/src/
cd ~/catkin_ws/
source ~/catkin_ws/devel/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
roslaunch dreamvu_pal_camera pal_rviz.launch

