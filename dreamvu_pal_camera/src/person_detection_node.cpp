#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> 
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/make_shared.hpp>

#include <fcntl.h>
#include <unistd.h>

#include "PAL.h"
#include "dreamvu_pal_camera/BoundingBox.h"
#include "dreamvu_pal_camera/BoundingBoxArray.h"

using namespace std;
using namespace cv;
using namespace PAL;

namespace PAL
{
	namespace Internal
	{
		void EnableDepth(bool flag);
	}
}

/*
Specify the absolute file path from which the settings are to be read.

If the specified file can't be opened, default properties from the API are used.
See PAL Documentation for more information.
*/
# define PROPERTIES_FILE_PATH "../catkin_ws/src/dreamvu_pal_camera/src/SavedPalProperties.txt"

static int camera_index = -1;
int width = -1;
int height = -1;

bool g_bRosOK = true;
cv::Mat g_imgLeft, g_imgRight, g_imgDepth;
PAL::CameraProperties g_CameraProperties;

image_transport::Publisher leftpub;
image_transport::Publisher depthpub;
ros::Publisher boxespub;

void publishimage(cv::Mat img, image_transport::Publisher& pub,string encoding)
{
        int type;
        if(encoding == "mono8")
            type = CV_8UC1;
        else if(encoding == "mono16")
            type = CV_16SC1;
        else
            type = CV_8UC3; 
                   
        sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), encoding, img).toImageMsg();
        pub.publish(imgmsg);
}




void OnLeftPanorama(cv::Mat* pLeft)
{
	g_imgLeft = *pLeft;

	publishimage(g_imgLeft, leftpub, "bgr8");
	
}

void OnDepthPanorama(cv::Mat *img)
{
	g_imgDepth = *img;

	//int depthSubnumber = depthpub.getNumSubscribers();
	//if (depthSubnumber > 0)
	{

		sensor_msgs::ImagePtr depthptr;
		depthptr.reset(new sensor_msgs::Image);

		depthptr->height = g_imgDepth.rows;
		depthptr->width = g_imgDepth.cols;

		int num = 1; // for endianness detection
		depthptr->is_bigendian = !(*(char*)&num == 1);

		depthptr->step = depthptr->width * sizeof(float);
		size_t size = depthptr->step * depthptr->height;
		depthptr->data.resize(size);

		depthptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

		memcpy((float*)(&depthptr->data[0]), g_imgDepth.data, size);

		depthpub.publish(depthptr);
	}
}


void OnPersonDetection(std::vector<PAL::BoundingBox> Boxes)
{


	dreamvu_pal_camera::BoundingBox box;
	dreamvu_pal_camera::BoundingBoxArray msg;

	for(int i=0; i<Boxes.size();i++)
	{
		box.x1=Boxes[i].x1; 
		box.x2=Boxes[i].x2; 
		box.y1=Boxes[i].y1; 
		box.y2=Boxes[i].y2; 
		msg.boxes.push_back(box);
	}

	boxespub.publish(msg);


}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "person_detection_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  //Creating all the publishers/subscribers
  leftpub = it.advertise("/dreamvu/pal/persons/get/left", 1);
  depthpub = it.advertise("/dreamvu/pal/persons/get/depth", 1);
  boxespub = nh.advertise<dreamvu_pal_camera::BoundingBoxArray>("/dreamvu/pal/persons/get/detection_boxes", 1);
  
  //PAL::Internal::EnableDepth(false);
  
  //Initialising PAL
  PAL::Acknowledgement ack1 = PAL::Init(width, height,camera_index);
  if(ack1 != PAL::SUCCESS)
  {
      return -1;
  }
 
  float threshold = 0.3f;

  PAL::Acknowledgement ack2 = PAL::InitPersonDetection(threshold);
  if(ack2 != PAL::SUCCESS)
  {
      return -1;
  }
  PAL::CameraProperties default_properties;
  PAL::GetCameraProperties(&default_properties);

  
  //Loading properties from the file
  PAL::Acknowledgement ack3 = PAL::LoadProperties(PROPERTIES_FILE_PATH,&g_CameraProperties);
  if(ack2 != PAL::SUCCESS)
  {

    ROS_WARN("Not able to load PAL settings from properties file at default location.\n\n" 
    "Please update the file location by setting the Macro: PROPERTIES_FILE_PATH in pal_camera_node.cpp and run catkin_make to build the package again.");
    ROS_INFO("Setting default properties to PAL.");
    g_CameraProperties = default_properties;
  }
  else
  {
    width = g_CameraProperties.resolution.width;
    height = g_CameraProperties.resolution.height;  
  }
  
  ros::Rate loop_rate(30);

  g_bRosOK = ros::ok();

  while(g_bRosOK)
  {
      
	//Getting no of subscribers for each publisher
	int leftSubnumber = leftpub.getNumSubscribers();
	int depthSubnumber = depthpub.getNumSubscribers();
	int boxesSubnumber = boxespub.getNumSubscribers();

	int totalSubnumber = leftSubnumber+depthSubnumber+boxesSubnumber;

	std::vector<PAL::BoundingBox> Boxes;  

	if(totalSubnumber) PAL::Acknowledgement ack2 = PAL::GetPeopleDetection(g_imgLeft, g_imgDepth, &Boxes);
	int num_of_persons = Boxes.size();

	for(int i=0; i<num_of_persons; i++)
	{
		cv::rectangle(g_imgLeft,cv::Point(Boxes[i].x1, Boxes[i].y1), cv::Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,255,0), 5);
	}

	if (boxesSubnumber > 0) OnPersonDetection(Boxes);
	if (leftSubnumber > 0) OnLeftPanorama(&g_imgLeft);
	if (depthSubnumber > 0) OnDepthPanorama(&g_imgDepth);

	Boxes.clear();
	ros::spinOnce();
	loop_rate.sleep();
	g_bRosOK = ros::ok();
  }

  return 0;
}
