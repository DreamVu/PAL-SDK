/*
	
	This sample subscriber can be used to getting detection bounding box values from PAL.
	It subscribes to a custom message of type dreamvu_pal_camera::BoundingBoxArray on /dreamvu/pal/persons/get/detection_boxes
	Please refer to PAL Documentation (Section 8) for more information.

*/

#include <ros/ros.h>
#include <sstream> 
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include <fcntl.h>
#include <unistd.h>

#include "PAL.h"
#include "dreamvu_pal_camera/BoundingBox.h"
#include "dreamvu_pal_camera/BoundingBoxArray.h"

using namespace std;
using namespace cv;

void on_detections(const dreamvu_pal_camera::BoundingBoxArray::ConstPtr& msg)
  {
    for (int i=0; i<msg->boxes.size(); ++i)
    {
      const dreamvu_pal_camera::BoundingBox &data = msg->boxes[i];
      ROS_INFO_STREAM("x1: " << data.x1 << "y1: " << data.y1 <<
                      "x2: " << data.x2 << "y2: " << data.y2);
    }
  }


int main(int argc, char **argv)
{
  
	ros::init(argc, argv, "sample_detection_sub");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/dreamvu/pal/persons/get/detection_boxes", 1, on_detections);

	ros::spin();


  


  return 0;
}

