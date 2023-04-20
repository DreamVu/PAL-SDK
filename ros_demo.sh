echo "Please select the launch file number:"
echo ""
echo "1 - scan_rviz : This will display the camera data in an Rviz/Rviz2 window with the left panorama along with the depth map and the 3D point cloud."
echo ""
echo "2 - object_tracking_rviz : This will display the left panorama in an Rviz/Rviz2 window with object tracking data overlayed on it along with the depth map."
echo ""
echo "3 - people_tracking_rviz : This will display the left panorama in an Rviz/Rviz2 window with people tracking data overlayed on it along with the depth map."
echo ""
echo "4 - object_detection_rviz : This will display the left panorama in an Rviz/Rviz2 window with object detection data overlayed on it along with the depth map."
echo ""
echo "5 - object_following_rviz : This will display the left panorama in an Rviz/Rviz2 window with object following data overlayed on it along with the depth map."
echo ""
echo "6 - people_following_rviz : This will display the left panorama in an Rviz/Rviz2 window with people following data overlayed on it along with the depth map."
echo ""
read -p "Enter the value: " arg1
cp -r dreamvu_pal_navigation ~/catkin_ws/src/
if [ -e /opt/ros/foxy/ ] 
then		
	if [ $arg1 == "1" ]; then
		cd ~/catkin_ws/
		colcon build
		. ./install/setup.bash
		colcon build
		ros2 launch src/dreamvu_pal_navigation/launch/scan_rviz_launch.py
	fi
	if [ $arg1 == "2" ]; then
		cd ~/catkin_ws/
		colcon build
		. ./install/setup.bash
		colcon build
		ros2 launch src/dreamvu_pal_navigation/launch/object_tracking_rviz_launch.py
	fi
	if [ $arg1 == "3" ]; then
		cd ~/catkin_ws/
		colcon build
		. ./install/setup.bash
		colcon build
		ros2 launch src/dreamvu_pal_navigation/launch/people_tracking_rviz_launch.py
	fi
	if [ $arg1 == "4" ]; then
		cd ~/catkin_ws/
		colcon build
		. ./install/setup.bash
		colcon build
		ros2 launch src/dreamvu_pal_navigation/launch/object_detection_rviz_launch.py
	fi
	if [ $arg1 == "5" ]; then
		cd ~/catkin_ws/
		colcon build
		. ./install/setup.bash
		colcon build
		ros2 launch src/dreamvu_pal_navigation/launch/object_following_rviz_launch.py
	fi
	if [ $arg1 == "6" ]; then
		cd ~/catkin_ws/
		colcon build
		. ./install/setup.bash
		colcon build
		ros2 launch src/dreamvu_pal_navigation/launch/people_following_rviz_launch.py
	fi
else
	if [ $arg1 == "1" ]; then	
		cd ~/catkin_ws/
		source ~/catkin_ws/devel/setup.bash
		catkin_make
		source ~/catkin_ws/devel/setup.bash
		roslaunch dreamvu_pal_navigation scan_rviz.launch
	fi
	if [ $arg1 == "2" ]; then	
		cd ~/catkin_ws/
		source ~/catkin_ws/devel/setup.bash
		catkin_make
		source ~/catkin_ws/devel/setup.bash
		roslaunch dreamvu_pal_navigation object_tracking_rviz.launch
	fi
	if [ $arg1 == "3" ]; then	
		cd ~/catkin_ws/
		source ~/catkin_ws/devel/setup.bash
		catkin_make
		source ~/catkin_ws/devel/setup.bash
		roslaunch dreamvu_pal_navigation people_tracking_rviz.launch
	fi
	if [ $arg1 == "4" ]; then	
		cd ~/catkin_ws/
		source ~/catkin_ws/devel/setup.bash
		catkin_make
		source ~/catkin_ws/devel/setup.bash
		roslaunch dreamvu_pal_navigation object_detection_rviz.launch
	fi
	if [ $arg1 == "5" ]; then	
		cd ~/catkin_ws/
		source ~/catkin_ws/devel/setup.bash
		catkin_make
		source ~/catkin_ws/devel/setup.bash
		roslaunch dreamvu_pal_navigation object_following_rviz.launch
	fi
	if [ $arg1 == "6" ]; then	
		cd ~/catkin_ws/
		source ~/catkin_ws/devel/setup.bash
		catkin_make
		source ~/catkin_ws/devel/setup.bash
		roslaunch dreamvu_pal_navigation people_following_rviz.launch
	fi
fi
