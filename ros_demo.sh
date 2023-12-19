echo "Please select the launch file number:"
echo ""
echo "1 - scan_rviz : This will display the camera data in an Rviz/Rviz2 window with the left panorama along with the depth map and the 3D point cloud."
echo ""
echo "2 - stereo_rviz : This will display the camera data in an Rviz/Rviz2 window with the high quality left & right panoramas."
echo ""
echo "3 - object_tracking_rviz : This will display the left panorama in an Rviz/Rviz2 window with object tracking data overlayed on it along with the depth map."
echo ""
echo "4 - people_tracking_rviz : This will display the left panorama in an Rviz/Rviz2 window with people tracking data overlayed on it along with the depth map."
echo ""
echo "5 - object_detection_rviz : This will display the left panorama in an Rviz/Rviz2 window with object detection data overlayed on it along with the depth map."
echo ""
echo "6 - object_following_rviz : This will display the left panorama in an Rviz/Rviz2 window with object following data overlayed on it along with the depth map."
echo ""
echo "7 - people_following_rviz : This will display the left panorama in an Rviz/Rviz2 window with people following data overlayed on it along with the depth map."
echo ""
read -p "Enter the value: " arg1
cp -r dreamvu_pal_navigation ~/catkin_ws/src/
non_foxy_launch_files=("scan_rviz.launch" "stereo_rviz.launch" "object_tracking_rviz.launch" "people_tracking_rviz.launch" "object_detection_rviz.launch" "object_following_rviz.launch" "people_following_rviz.launch")
foxy_launch_files=("scan_rviz_launch.py" "stereo_rviz_launch.py" "object_tracking_rviz_launch.py" "people_tracking_rviz_launch.py" "object_detection_rviz_launch.py" "object_following_rviz_launch.py" "people_following_rviz_launch.py") 
humble_launch_files=("scan_rviz_launch.py" "stereo_rviz_launch.py" "object_tracking_rviz_launch.py" "people_tracking_rviz_launch.py" "object_detection_rviz_launch.py" "object_following_rviz_launch.py" "people_following_rviz_launch.py") 
if [ -e /opt/ros/foxy/ ] 
then		
	cd ~/catkin_ws/
	colcon build
	. ./install/setup.bash
	colcon build
	ros2 launch src/dreamvu_pal_navigation/launch/${foxy_launch_files[arg1-1]}
	
elif [ -e /opt/ros/humble/ ]
then
	cd ~/catkin_ws/
	colcon build
	. ./install/setup.bash
	colcon build
	ros2 launch src/dreamvu_pal_navigation/launch/${humble_launch_files[arg1-1]}
	
else
	cd ~/catkin_ws/
	source ~/catkin_ws/devel/setup.bash
	catkin_make
	source ~/catkin_ws/devel/setup.bash
	roslaunch dreamvu_pal_navigation ${non_foxy_launch_files[arg1-1]}
fi
