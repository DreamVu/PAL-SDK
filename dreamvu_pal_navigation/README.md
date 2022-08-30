ROS Wrapper for PAL

- First download PAL SDK and run install script.

- Move dreamvu_pal_navigation package in the src directory of catkin workspace(/catkin_ws/src).

- Open a terminal and build the package:

        $ cd ~/catkin_ws
        $ catkin_make
        $ source ./devel/setup.bash
        
- To launch the camera node along with an Rviz preview, open a terminal from catkin workspace and run:
        
        $ source ./devel/setup.bash
        $ roslaunch dreamvu_pal_navigation scan_rviz.launch
       
