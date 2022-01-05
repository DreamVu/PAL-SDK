#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp" 
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

using namespace std;
using namespace cv;
using namespace PAL;


/*
Specify the absolute file path from which the settings are to be read.

If the specified file can't be opened, default properties from the API are used.
See PAL Documentation for more information.
*/
# define PROPERTIES_FILE_PATH "../dev_ws/src/dreamvu_pal_camera/src/SavedPalProperties.txt"

static int camera_index = -1;
int width = -1;
int height = -1;

bool g_bRosOK = true;
PAL::Image g_imgLeft, g_imgRight, g_imgDepth;
PAL::CameraProperties g_CameraProperties;

image_transport::Publisher leftpub;
image_transport::Publisher rightpub;

image_transport::Publisher depthpub;
// rclcpp::Publisher::SharedPtr pointcloudPub;

void publishimage(PAL::Image img, image_transport::Publisher& pub,string encoding)
{
        int type;
        if(encoding == "mono8")
            type = CV_8UC1;
        else if(encoding == "mono16")
            type = CV_16SC1;
        else
            type = CV_8UC3; 
                   
        cv::Mat imgmat = cv::Mat(img.rows, img.cols, type, img.Raw.u8_data);
		auto  header = std_msgs::msg::Header();
		header.frame_id = "pal";
        sensor_msgs::msg::Image_<std::allocator<void> >::SharedPtr  imgmsg = cv_bridge::CvImage(header, encoding, imgmat).toImageMsg();
        // imgmsg.frame_id = "left";
		pub.publish(imgmsg);
}

void OnLeftPanorama(PAL::Image* pLeft)
{
	g_imgLeft = *pLeft;

	//int leftSubnumber = leftpub.getNumSubscribers();
	// Publish the g_imgLeft panorama if someone has subscribed to
	//if (leftSubnumber > 0)
	{
		if (g_CameraProperties.color_space == PAL::YUV444)
		{
			cv::Mat l(g_imgLeft.rows, g_imgLeft.cols, CV_8UC3, g_imgLeft.Raw.u8_data);
			cv::cvtColor(l, l, CV_YUV2BGR);
			g_imgLeft.Set(l.data, l.cols, l.rows);
		}
		publishimage(g_imgLeft, leftpub, "bgr8");
	}
}

void OnRightPanorama(PAL::Image* pRight)
{
	g_imgRight = *pRight;

	//int rightSubnumber = rightpub.getNumSubscribers();
	// Publish the g_imgRight panorama if someone has subscribed to
	//if (rightSubnumber > 0)
	{
		if (g_CameraProperties.color_space == PAL::YUV444)
		{
			cv::Mat r(g_imgRight.rows, g_imgRight.cols, CV_8UC3, g_imgRight.Raw.u8_data);
			cv::cvtColor(r, r, CV_YUV2BGR);
			g_imgRight.Set(r.data, r.cols, r.rows);
		}
		publishimage(g_imgRight, rightpub, "bgr8");
	}
}


void OnDepthPanorama(PAL::Image *img)
{
	g_imgDepth = *img;

	//int depthSubnumber = depthpub.getNumSubscribers();
	//if (depthSubnumber > 0)
	{

		sensor_msgs::msg::Image depthptr;

		depthptr.height = g_imgDepth.rows;
		depthptr.width = g_imgDepth.cols;
		depthptr.header.frame_id = "pal";
		int num = 1; // for endianness detection
		depthptr.is_bigendian = !(*(char*)&num == 1);
		depthptr.step = depthptr.width * sizeof(float);
		size_t size = depthptr.step * depthptr.height;
		depthptr.data.resize(size);

		depthptr.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		memcpy((float*)(&depthptr.data[0]), g_imgDepth.Raw.data, size);

		depthpub.publish(depthptr);
	}
}



int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pal_camera_node");

  image_transport::ImageTransport it(node);
  
  //Creating all the publishers/subscribers
  leftpub = it.advertise("/dreamvu/pal/get/left", 1);
  rightpub = it.advertise("/dreamvu/pal/get/right", 1);
  depthpub = it.advertise("/dreamvu/pal/get/depth", 1);
  auto pointcloudPub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/dreamvu/pal/get/point_cloud", 1);


  //Initialising PAL
  PAL::Acknowledgement ack1 = PAL::Init(width, height,camera_index);
  if(ack1 != PAL::SUCCESS)
  {
      return -1;
  }

  PAL::CameraProperties default_properties;
  PAL::GetCameraProperties(&default_properties);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), PROPERTIES_FILE_PATH);
  //Loading properties from the file
  PAL::Acknowledgement ack2 = PAL::LoadProperties(PROPERTIES_FILE_PATH,&g_CameraProperties);
  if(ack2 != PAL::SUCCESS)
  {

    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Not able to load PAL settings from properties file at default location.\n\n" 
    "Please update the file location by setting the Macro: PROPERTIES_FILE_PATH in pal_camera_node.cpp and build the package again.");
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
    int rightSubnumber = rightpub.getNumSubscribers();
	int depthSubnumber = depthpub.getNumSubscribers();
	int pointcloudSubnumber = pointcloudPub->get_subscription_count();

	if (pointcloudSubnumber > 0)
	{
		std::vector<PAL::Point> pc;
		sensor_msgs::msg::PointCloud2 pointcloudMsg;
		// rclcpp::WallTime t1 = ros::WallTime::now();

		PAL::Acknowledgement ack = PAL::GetPointCloud(&pc, 0, &g_imgLeft, &g_imgRight, &g_imgDepth, 0);

		// rclcpp::WallTime t2 = ros::WallTime::now();


		std::vector<PAL::Point>* point_data = &pc;

		pointcloudMsg.is_bigendian = false;
		pointcloudMsg.is_dense = false;

		sensor_msgs::PointCloud2Modifier modifier(pointcloudMsg);
		pointcloudMsg.point_step = 4 * sizeof(float);

		pointcloudMsg.width = point_data->size();
		pointcloudMsg.height = 1;
		pointcloudMsg.row_step = sizeof(PAL::Point) * point_data->size();
		pointcloudMsg.header.frame_id = "pal";

		modifier.setPointCloud2Fields(4,
			"x", 1, sensor_msgs::msg::PointField::FLOAT32,
			"y", 1, sensor_msgs::msg::PointField::FLOAT32,
			"z", 1, sensor_msgs::msg::PointField::FLOAT32,
			"rgb", 1, sensor_msgs::msg::PointField::FLOAT32
		);


		PAL::Point* pointcloudPtr = (PAL::Point*)(&pointcloudMsg.data[0]);

		unsigned long int i;
		PAL::Point *pc_points = &pc[0];
		for (i = 0; i < point_data->size(); i++)
		{
			pointcloudPtr[i].a = pc_points[i].a;
			pointcloudPtr[i].g = pc_points[i].g;
			pointcloudPtr[i].b = pc_points[i].r;
			pointcloudPtr[i].r = pc_points[i].b;

			//unit conversion from centimeter to meter
			pointcloudPtr[i].x = (pc_points[i].x) *0.01;
			pointcloudPtr[i].z = (pc_points[i].y)  *0.01;
			pointcloudPtr[i].y = -(pc_points[i].z) *0.01;

		}

		// rclcpp::WallTime t3 = ros::WallTime::now();


		pointcloudPub->publish(pointcloudMsg);

		// rclcpp::WallTime t4 = ros::WallTime::now();

		//ROS_INFO_STREAM("Grab time (ms): " << (t2 - t1).toNSec()*1e-6);
		//ROS_INFO_STREAM("convert time (ms): " << (t3 - t2).toNSec()*1e-6);
		//ROS_INFO_STREAM("Publish time (ms): " << (t4 - t3).toNSec()*1e-6);
	}
	else
	{
		PAL::Acknowledgement ack2 = GrabFrames(&g_imgLeft, &g_imgRight, depthSubnumber ? &g_imgDepth : 0,
			0, false, depthSubnumber ? false : true);
	}
    
	if (leftSubnumber > 0) OnLeftPanorama(&g_imgLeft);
	if (rightSubnumber > 0) OnRightPanorama(&g_imgRight);
	if (depthSubnumber > 0) OnDepthPanorama(&g_imgDepth);
   
    rclcpp::spin_some(node);
    loop_rate.sleep();
	g_bRosOK = rclcpp::ok();
  }

  return 0;
}
