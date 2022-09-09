cp -r dreamvu_pal_navigation ~/catkin_ws/src/
if [ -e /opt/ros/foxy/ ] 
then
	cd ~/catkin_ws/
	colcon build
	. ./install/setup.bash
	colcon build
	ros2 launch src/dreamvu_pal_navigation/launch/scan_rviz_launch.py
else
	cd ~/catkin_ws/
	source ~/catkin_ws/devel/setup.bash
	catkin_make
	source ~/catkin_ws/devel/setup.bash
	roslaunch dreamvu_pal_navigation scan_rviz.launch
fi
