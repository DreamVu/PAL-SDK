#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp" /*instead of ros.h*/
#include "rclcpp/publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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
#include "dreamvu_pal_camera/msg/bounding_box.hpp"
#include "dreamvu_pal_camera/msg/bounding_box_array.hpp"

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
// ros::Publisher boxespub;

void publishimage(cv::Mat img, image_transport::Publisher& pub,string encoding)
{
        int type;
        if(encoding == "mono8")
            type = CV_8UC1;
        else if(encoding == "mono16")
            type = CV_16SC1;
        else
            type = CV_8UC3; 
        auto  header = std_msgs::msg::Header();
		header.frame_id = "pal";       
        sensor_msgs::msg::Image_<std::allocator<void> >::SharedPtr imgmsg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
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

		sensor_msgs::msg::Image depthptr;
		// depthptr.reset(new sensor_msgs::Image);

		depthptr.height = g_imgDepth.rows;
		depthptr.width = g_imgDepth.cols;
		depthptr.header.frame_id = "pal";
		int num = 1; // for endianness detection
		depthptr.is_bigendian = !(*(char*)&num == 1);

		depthptr.step = depthptr.width * sizeof(float);
		size_t size = depthptr.step * depthptr.height;
		depthptr.data.resize(size);

		depthptr.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

		memcpy((float*)(&depthptr.data[0]), g_imgDepth.data, size);

		depthpub.publish(depthptr);
	}
}


void OnPersonDetection(std::vector<PAL::BoundingBox> Boxes, std::shared_ptr<rclcpp::Publisher<dreamvu_pal_camera::msg::BoundingBoxArray_<std::allocator<void> >, std::allocator<void> > > boxespub)
{


	dreamvu_pal_camera::msg::BoundingBox box;
	dreamvu_pal_camera::msg::BoundingBoxArray msg;

	for(int i=0; i<Boxes.size();i++)
	{
		box.x1=Boxes[i].x1; 
		box.x2=Boxes[i].x2; 
		box.y1=Boxes[i].y1; 
		box.y2=Boxes[i].y2; 
		msg.boxes.push_back(box);
	}

	boxespub->publish(msg);


}

int main(int argc, char** argv)
{

//   ros::init(argc, argv, "person_detection_node");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("person_detection_node");
//   ros::NodeHandle nh;
//   image_transport::ImageTransport it(nh);
  image_transport::ImageTransport it(node);

  //Creating all the publishers/subscribers
  leftpub = it.advertise("/dreamvu/pal/persons/get/left", 1);
  depthpub = it.advertise("/dreamvu/pal/persons/get/depth", 1);
//   boxespub = nh.advertise<dreamvu_pal_camera::BoundingBoxArray>("/dreamvu/pal/persons/get/detection_boxes", 1);
  auto boxespub = node->create_publisher<dreamvu_pal_camera::msg::BoundingBoxArray>("/dreamvu/pal/persons/get/detection_boxes", 1);
  
  //PAL::Internal::EnableDepth(false);
  
  //Initialising PAL
  PAL::Acknowledgement ack1 = PAL::Init(width, height,camera_index);
  if(ack1 != PAL::SUCCESS)
  {
      return -1;
  }
 
  float threshold = 0.5f;

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

    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Not able to load PAL settings from properties file at default location.\n\n" 
    "Please update the file location by setting the Macro: PROPERTIES_FILE_PATH in pal_camera_node.cpp and run catkin_make to build the package again.");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting default properties to PAL.");
    g_CameraProperties = default_properties;
  }
  else
  {
    width = g_CameraProperties.resolution.width;
    height = g_CameraProperties.resolution.height;  
  }
  
  rclcpp::Rate loop_rate(30);

  g_bRosOK = rclcpp::ok();

  while(g_bRosOK)
  {
      
	//Getting no of subscribers for each publisher
	int leftSubnumber = leftpub.getNumSubscribers();
	int depthSubnumber = depthpub.getNumSubscribers();
	int boxesSubnumber = boxespub->get_subscription_count();

	int totalSubnumber = leftSubnumber+depthSubnumber+boxesSubnumber;

	std::vector<PAL::BoundingBox> Boxes;  

	if(totalSubnumber) PAL::Acknowledgement ack2 = PAL::GetPeopleDetection(g_imgLeft, g_imgDepth, &Boxes);
	int num_of_persons = Boxes.size();

	for(int i=0; i<num_of_persons; i++)
	{
		cv::rectangle(g_imgLeft,cv::Point(Boxes[i].x1, Boxes[i].y1), cv::Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,255,0), 5);
	}

	if (boxesSubnumber > 0) OnPersonDetection(Boxes, boxespub);
	if (leftSubnumber > 0) OnLeftPanorama(&g_imgLeft);
	if (depthSubnumber > 0) OnDepthPanorama(&g_imgDepth);

	Boxes.clear();
	rclcpp::spin_some(node);
	loop_rate.sleep();
	g_bRosOK = rclcpp::ok();
  }

  return 0;
}
