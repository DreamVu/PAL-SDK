#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/make_shared.hpp>
#include <sys/time.h>

#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <thread>

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "PAL.h"
#include "filters/filter_chain.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


using namespace std;
using namespace cv;
using namespace PAL;

static const float Pi = 3.1415926535898f;

/*
Specify the absolute file path from which the settings are to be read.

If the specified file can't be opened, default properties from the API are used.
See PAL Mini Documentation for more information.
*/
#define PROPERTIES_FILE_PATH "../catkin_ws/src/dreamvu_pal_navigation/src/SavedPalProperties.txt"
                              

static int camera_index = -1;
int width = -1;
int height = -1;

bool g_bRosOK = true;

PAL::CameraProperties g_CameraProperties;

image_transport::Publisher leftpub1, leftpub2, depthPub1, depthPub2, floorPub1, floorPub2, stereoleftpub1, stereorightpub1, stereoleftpub2, stereorightpub2;

ros::Publisher laserPub1, laserPub2;
ros::Publisher pointcloudPub1, pointcloudPub2;

filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");

void publishLaser(cv::Mat scan_mat, ros::Publisher laserPub, timeval timestamp, std::string frame_id)
{
	
	sensor_msgs::LaserScan scan, scan1;
    //scan.header.stamp = timestamp;
	scan.header.stamp.sec = timestamp.tv_sec;
	scan.header.stamp.nsec = timestamp.tv_usec*1000;

	scan.header.frame_id = frame_id;
	scan.angle_min = Pi / 5;
	scan.angle_max = 2 * Pi + scan.angle_min;
	scan.angle_increment = 6.28 / scan_mat.cols;
	scan.range_min = 0.0;
	scan.range_max = 50.0;
	scan.ranges.resize(scan_mat.cols);
	scan.intensities.resize(scan_mat.cols);

	float* pscan = (float*) scan_mat.data;

	for (int i = 0; i < scan_mat.cols; i++)
	{
	    scan.ranges[i] = *(pscan+i);
	    scan.intensities[i] = 0.5;	
	}

    //filter_chain_.update(scan, scan);
	laserPub.publish(scan);

}

Mat getColorMap(Mat img, float scale)
{
    Mat img_new = img.clone();
    img_new.convertTo(img_new, CV_8UC1);
    
    cv::Mat invert = 255-img_new;
    cv::Mat out;    
    applyColorMap(invert, out, COLORMAP_JET);
    return out;
}

void OnDepthPanorama(cv::Mat img, image_transport::Publisher depthPub)
{

	cv::Mat colored = getColorMap(img, 1);
    std_msgs::Header header;
    sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(header, "bgr8", colored).toImageMsg();
	depthPub.publish(imgmsg);	
	
}

void publishimage(cv::Mat imgmat, image_transport::Publisher &pub, string encoding, timeval timestamp)
{
	int type;
	if (encoding == "mono8")
		type = CV_8UC1;
	else if (encoding == "mono16")
		type = CV_16SC1;
	else
		type = CV_8UC3;

    std_msgs::Header header;
    header.stamp.sec = timestamp.tv_sec;
    header.stamp.nsec = timestamp.tv_usec*1000;
    sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(header, encoding, imgmat).toImageMsg();

	pub.publish(imgmsg);
}

std::vector<PAL::Data::ODOA_Data> data1;

bool overlaid1 = false;
bool overlaid2 = false;

bool updatePC = false;

void PublishPC(cv::Mat pcMat, ros::Publisher pointcloudPub, std::string frame_id)
{	
	std::vector<PAL::Point> pc;
	sensor_msgs::PointCloud2Ptr pointcloudMsg;
	pointcloudMsg.reset(new sensor_msgs::PointCloud2);
	ros::WallTime t1 = ros::WallTime::now();


	PAL::Point* pc_points = (PAL::Point*) pcMat.data;
	long int size = pcMat.rows*pcMat.cols;
	pointcloudMsg->is_bigendian = false;
	pointcloudMsg->is_dense = false;

	sensor_msgs::PointCloud2Modifier modifier(*pointcloudMsg);
	pointcloudMsg->point_step = 4 * sizeof(float);

	pointcloudMsg->width = size;
	pointcloudMsg->height = 1;
	pointcloudMsg->row_step = sizeof(PAL::Point) * size;
	pointcloudMsg->header.frame_id = frame_id;

	modifier.setPointCloud2Fields(4,
		"x", 1, sensor_msgs::PointField::FLOAT32,
		"y", 1, sensor_msgs::PointField::FLOAT32,
		"z", 1, sensor_msgs::PointField::FLOAT32,
		"rgb", 1, sensor_msgs::PointField::FLOAT32
	);


	PAL::Point* pointcloudPtr = (PAL::Point*)(&pointcloudMsg->data[0]);

	unsigned long int i;

	for (i = 0; i < size; i++)
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

	ros::WallTime t3 = ros::WallTime::now();


	pointcloudPub.publish(pointcloudMsg);

}


static void RunPC1Thread()
{
	while(g_bRosOK)
	{
	
		int pointcloudSubnumber = pointcloudPub1.getNumSubscribers();
		ros::WallTime start = ros::WallTime::now();
								
  		if(pointcloudSubnumber > 0)
		{
			PublishPC(data1[0].point_cloud, pointcloudPub1, "pal");
		}
		
		usleep(50000);
		
		ros::WallTime end = ros::WallTime::now();						
		//ROS_INFO_STREAM("PublishPC 1 (ms): " << (end - start).toNSec()*1e-6);		
	}	
}

static void RunPC2Thread()
{
	while(g_bRosOK)
	{	
		int pointcloudSubnumber = pointcloudPub2.getNumSubscribers();
		ros::WallTime start = ros::WallTime::now();
								
  		if(pointcloudSubnumber > 0)
		{
			PublishPC(data1[1].point_cloud, pointcloudPub2, "pal_1");
		}
		
		usleep(50000);
		
		ros::WallTime end = ros::WallTime::now();						
		//ROS_INFO_STREAM("PublishPC 2 (ms): " << (end - start).toNSec()*1e-6);		
	}	
}
static void RunThread()
{

	ros::Rate loop_rate2(5);
	while(g_bRosOK)
	{

  		int left2Subnumber = leftpub2.getNumSubscribers();		
		int laserscan2Subnumber = laserPub2.getNumSubscribers();
		int depth2Subnumber = depthPub2.getNumSubscribers();
		int floor2Subnumber = floorPub2.getNumSubscribers();				
		int stereoleft2Subnumber = stereoleftpub2.getNumSubscribers();				
		int stereoright2Subnumber = stereorightpub2.getNumSubscribers();						
		
		ros::WallTime start = ros::WallTime::now();	
									
		if (left2Subnumber+laserscan2Subnumber+depth2Subnumber+floor2Subnumber+stereoleft2Subnumber+stereoright2Subnumber > 0)		
		{
			overlaid2 = (left2Subnumber && laserscan2Subnumber);       

		    if (left2Subnumber > 0)
			{
		        publishimage(overlaid2 ? data1[1].marked_left  : data1[1].left, leftpub2, "bgr8", data1[1].timestamp);
		    }
			if (stereoleft2Subnumber > 0)
			{
		        publishimage(data1[1].left, stereoleftpub2, "bgr8", data1[1].timestamp);	
		    }        
			if (stereoright2Subnumber > 0)
			{
		        publishimage(data1[1].right, stereorightpub2, "bgr8", data1[1].timestamp);	
		    }		    
			if (laserscan2Subnumber > 0)
			{
				publishLaser(data1[1].scan, laserPub2, data1[1].timestamp, "pal_1");
			}
			if (depth2Subnumber > 0)
			{
				OnDepthPanorama(data1[1].distance, depthPub2);
			}		
			if (floor2Subnumber > 0)
			{
		        publishimage(data1[1].de_out, floorPub2, "mono8", data1[1].timestamp);
		    }  
		}
		
		ros::spinOnce();
		loop_rate2.sleep();
		g_bRosOK = ros::ok();
	
		ros::WallTime end = ros::WallTime::now();						
		//ROS_INFO_STREAM("Grab time 2 (ms): " << (end - start).toNSec()*1e-6);
		
	}	
}

std::thread m_oThreadID, m_oThreadID1, m_oThreadID2;

void Start2()
{
        m_oThreadID = std::thread(RunThread);
        m_oThreadID1 = std::thread(RunPC1Thread);
        m_oThreadID2 = std::thread(RunPC2Thread);        
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "depth_scan_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    filter_chain_.configure("/scan_filter_chain", nh);
    
	//Creating all the publishers
	leftpub1 = it.advertise("/dreamvu/pal/odoa/get/left1", 1);
	leftpub2 = it.advertise("/dreamvu/pal/odoa/get/left2", 1);
	
	stereoleftpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/left1", 1);		
	stereorightpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/right1", 1);				

	stereoleftpub2 = it.advertise("/dreamvu/pal/odoa/get/stereo/left2", 1);		
	stereorightpub2 = it.advertise("/dreamvu/pal/odoa/get/stereo/right2", 1);				

	depthPub1 = it.advertise("/dreamvu/pal/odoa/get/depth1", 1);
	depthPub2 = it.advertise("/dreamvu/pal/odoa/get/depth2", 1);
	
	floorPub1 = it.advertise("/dreamvu/pal/odoa/get/ground1", 1);    			
	floorPub2 = it.advertise("/dreamvu/pal/odoa/get/ground2", 1);
		    	
	laserPub1 = nh.advertise<sensor_msgs::LaserScan>("/dreamvu/pal/odoa/get/scan1", 1);  
	laserPub2 = nh.advertise<sensor_msgs::LaserScan>("/dreamvu/pal/odoa/get/scan2", 1);

    pointcloudPub1 = nh.advertise<sensor_msgs::PointCloud2>("/dreamvu/pal/odoa/get/point_cloud1", 1);
    pointcloudPub2 = nh.advertise<sensor_msgs::PointCloud2>("/dreamvu/pal/odoa/get/point_cloud2", 1); 

    int width, height;
    
	std::vector<int> camera_indexes{5,6};

	PAL::Mode def_mode = PAL::Mode::LASER_SCAN;

	char path[1024];
	sprintf(path,"/data%d/",camera_indexes[0]);

	char path2[1024];
	sprintf(path2,"/data%d/",camera_indexes[1]);

	PAL::SetPathtoData(path, path2);

	if (PAL::Init(width, height, camera_indexes, &def_mode) != PAL::SUCCESS) //Connect to the PAL camera
	{
		cout<<"Init failed"<<endl;
		return 1;
	}
	
    PAL::SetAPIMode(PAL::API_Mode::ALL_MODE);
	
	//Loading properties from the file
	PAL::Acknowledgement ack_load1 = PAL::LoadProperties(PROPERTIES_FILE_PATH, &g_CameraProperties);
	
	if (ack_load1 != PAL::SUCCESS)
	{

		ROS_WARN("Not able to load PAL Mini settings from properties file at default location.\n\n"
				 "Please update the file location by setting the Macro: PROPERTIES_FILE_PATH in multi_depth_scan_node.cpp and run catkin_make to build the package again.");
		ROS_INFO("Setting default properties to PAL Mini.");

	}

	ros::Rate loop_rate(30);
	g_bRosOK = ros::ok();
    

    for(int i=0; i<10; i++)
    {
    	data1 = PAL::GrabRangeScanData();
    }
    
    Start2();

	while (g_bRosOK)
	{

		//Getting no of subscribers for each publisher
		int left1Subnumber = leftpub1.getNumSubscribers();
		int laserscan1Subnumber = laserPub1.getNumSubscribers();
		int depth1Subnumber = depthPub1.getNumSubscribers();
		int floor1Subnumber = floorPub1.getNumSubscribers();				
		int stereoleft1Subnumber = stereoleftpub1.getNumSubscribers();				
		int stereoright1Subnumber = stereorightpub1.getNumSubscribers();						
		
		int subnumber = left1Subnumber+laserscan1Subnumber+stereoleft1Subnumber+stereoright1Subnumber;//left2Subnumber + laserscan2Subnumber;
        
		if (subnumber > 0)
		{
			ros::WallTime t1 = ros::WallTime::now();
			overlaid1 = (left1Subnumber && laserscan1Subnumber);       

            data1 = PAL::GrabRangeScanData();
			ros::WallTime t2 = ros::WallTime::now();						
			//ROS_INFO_STREAM("Grab time 1 (ms): " << (t2 - t1).toNSec()*1e-6);					
					
		}
		if (left1Subnumber > 0)
		{
            publishimage(overlaid1 ? data1[0].marked_left  : data1[0].left, leftpub1, "bgr8", data1[0].timestamp);	 	
        }
		if (stereoleft1Subnumber > 0)
		{
            publishimage(data1[0].left, stereoleftpub1, "bgr8", data1[0].timestamp);	
        }        
		if (stereoright1Subnumber > 0)
		{
            publishimage(data1[0].right, stereorightpub1, "bgr8", data1[0].timestamp);	
        }
		if (laserscan1Subnumber > 0)
		{	
			publishLaser(data1[0].scan, laserPub1, data1[0].timestamp, "pal");
		}		
		if (depth1Subnumber > 0)
		{
			OnDepthPanorama(data1[0].distance, depthPub1);
		}		
		if (floor1Subnumber > 0)
		{
            publishimage(data1[0].de_out, floorPub1, "mono8", data1[0].timestamp);
        }
        
		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	m_oThreadID.join();
	m_oThreadID1.join();
	m_oThreadID2.join();		
	PAL::Destroy();

}
