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
#include <thread>

#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <math.h>

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

image_transport::Publisher leftpub1, stereoleftpub1, stereorightpub1, depthPub1,floorPub;
ros::Publisher laserPub1;
ros::Publisher pointcloudPub1;

filters::FilterChain<sensor_msgs::LaserScan> filter_chain_("sensor_msgs::LaserScan");
std::vector<PAL::Data::ODOA_Data> data1;
bool updatePC = false;
int pointcloudSubnumber=0;

Mat getColorMap(Mat img, float scale)
{
    Mat img_new = img * scale;
    img_new.convertTo(img_new, CV_8UC1);
    img_new = 255-img_new;
    applyColorMap(img_new, img_new, COLORMAP_JET);
    return img_new;
}

void OnDepthPanorama(cv::Mat img)
{

	cv::Mat colored = getColorMap(img, 1);	
    std_msgs::Header header;
    sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(header, "bgr8", colored).toImageMsg();
	depthPub1.publish(imgmsg);
		
}

void PublishPC(cv::Mat pcMat)
{

	cv::Mat newpcMat;
	
	#if 0
		cv::resize(pcMat, newpcMat, cv::Size(pcMat.rows, pcMat.rows), 1, 1, INTER_NEAREST);
	#else
		newpcMat = pcMat;
	#endif
	
	std::vector<PAL::Point> pc;
	sensor_msgs::PointCloud2Ptr pointcloudMsg;
	pointcloudMsg.reset(new sensor_msgs::PointCloud2);
	ros::WallTime t1 = ros::WallTime::now();


	ros::WallTime t2 = ros::WallTime::now();


	PAL::Point* pc_points = (PAL::Point*) newpcMat.data;
	long int size = newpcMat.rows*newpcMat.cols;
	pointcloudMsg->is_bigendian = false;
	pointcloudMsg->is_dense = false;
 
	sensor_msgs::PointCloud2Modifier modifier(*pointcloudMsg);
	pointcloudMsg->point_step = 4 * sizeof(float);

	pointcloudMsg->width = size;
	pointcloudMsg->height = 1;
	pointcloudMsg->row_step = sizeof(PAL::Point) * size;
	pointcloudMsg->header.frame_id = "pal";

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


	pointcloudPub1.publish(pointcloudMsg);

}


static void RunThread()
{

	while(g_bRosOK)
	{
	
		pointcloudSubnumber = pointcloudPub1.getNumSubscribers();
		ros::WallTime start = ros::WallTime::now();
								
  		if(pointcloudSubnumber > 0)
		{
			PublishPC(data1[0].point_cloud);
		}
		usleep(20000);
		
		ros::WallTime end = ros::WallTime::now();						
		//ROS_INFO_STREAM("Grab time 2 (ms): " << (end - start).toNSec()*1e-6);
		
	}	
}
void publishLaser(cv::Mat scan_mat, ros::Publisher laserPub, timeval timestamp)
{
	
	sensor_msgs::LaserScan scan, scan1;
    //scan.header.stamp = timestamp;
	scan.header.stamp.sec = timestamp.tv_sec;
	scan.header.stamp.nsec = timestamp.tv_usec*1000;

	scan.header.frame_id = "pal";
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

    filter_chain_.update(scan, scan);
	laserPub.publish(scan);

}

std::thread m_oThreadID;

void Start2()
{
        m_oThreadID = std::thread(RunThread);
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


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "depth_scan_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    filter_chain_.configure("/scan_filter_chain", nh);
	//Creating all the publishers
	leftpub1 = it.advertise("/dreamvu/pal/odoa/get/left", 1);		
	stereoleftpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/left", 1);		
	stereorightpub1 = it.advertise("/dreamvu/pal/odoa/get/stereo/right", 1);				
	laserPub1 = nh.advertise<sensor_msgs::LaserScan>("/dreamvu/pal/odoa/get/scan", 1);  
    pointcloudPub1 = nh.advertise<sensor_msgs::PointCloud2>("/dreamvu/pal/odoa/get/point_cloud", 1);
	depthPub1 = it.advertise("/dreamvu/pal/odoa/get/depth", 1);  
	floorPub = it.advertise("/dreamvu/pal/odoa/get/ground", 1);    
	
	int width, height;
    std::vector<int> camera_indexes{5};
    
    PAL::Mode init_mode = PAL::Mode::LASER_SCAN;
	
	char path[1024];
	sprintf(path,"/data%d/",camera_indexes[0]);

	char path2[1024];
	sprintf(path2,"/data%d/",6);

	PAL::SetPathtoData(path, path2);

	if (PAL::Init(width, height, camera_indexes, &init_mode) != PAL::SUCCESS) //Connect to the PAL Mini camera
	{
		cout<<"Init failed"<<endl;
		return 1;
	}

	//cout<<"Init"<<endl;
	usleep(100000);
    PAL::SetAPIMode(PAL::API_Mode::ALL_MODE);

	//Loading properties from the file
	PAL::Acknowledgement ack_load1 = PAL::LoadProperties(PROPERTIES_FILE_PATH, &g_CameraProperties);
	usleep(100000);
	
	if (ack_load1 != PAL::SUCCESS)
	{

		ROS_WARN("Not able to load PAL Mini settings from properties file at default location.\n\n"
				 "Please update the file location by setting the Macro: PROPERTIES_FILE_PATH in depth_scan_node.cpp and run catkin_make to build the package again.");
		ROS_INFO("Setting default properties to PAL Mini.");

	}


	ros::Rate loop_rate(20);
	g_bRosOK = ros::ok();
    

    for(int i=0; i<10; i++)
    {
    	data1 = PAL::GrabRangeScanData();
    }
    
    Start2();
    
	while (g_bRosOK)
	{

		int g_emode = 0;

		//Getting no of subscribers for each publisher
		int left1Subnumber = leftpub1.getNumSubscribers();
		int laserscan1Subnumber = laserPub1.getNumSubscribers();
		int depth1Subnumber = depthPub1.getNumSubscribers();
		int floorSubnumber = floorPub.getNumSubscribers();
		int stereoleft1Subnumber = stereoleftpub1.getNumSubscribers();				
		int stereoright1Subnumber = stereorightpub1.getNumSubscribers();

		int subnumber = left1Subnumber+laserscan1Subnumber+pointcloudSubnumber+floorSubnumber+stereoright1Subnumber+stereoleft1Subnumber;
        bool overlaid1 = false;
        updatePC = false;

		if (left1Subnumber > 0)
		{
            publishimage(data1[0].marked_left, leftpub1, "bgr8", data1[0].timestamp);
            g_emode = g_emode | PAL::API_Mode::RANGE_SCAN;	
        }
		if (stereoleft1Subnumber > 0)
		{
            publishimage(data1[0].left, stereoleftpub1, "bgr8", data1[0].timestamp);
            g_emode = g_emode | PAL::API_Mode::STEREO;	            	
        }        
		if (stereoright1Subnumber > 0)
		{
            publishimage(data1[0].right, stereorightpub1, "bgr8", data1[0].timestamp);	
            g_emode = g_emode | PAL::API_Mode::STEREO;	            	

        }
		if (laserscan1Subnumber > 0)
		{
			publishLaser(data1[0].scan, laserPub1, data1[0].timestamp);
            g_emode = g_emode | PAL::RANGE_SCAN;	            	

		}		
		if (depth1Subnumber > 0)
		{
			OnDepthPanorama(data1[0].distance);
            g_emode = g_emode | PAL::API_Mode::DEPTH;	            	
		}		
		if (floorSubnumber > 0)
		{
            publishimage(data1[0].de_out, floorPub, "mono8", data1[0].timestamp);
            g_emode = g_emode | PAL::RANGE_SCAN;	            	
        }
         
        if(pointcloudSubnumber)
        {
            g_emode = g_emode | PAL::API_Mode::POINT_CLOUD;	            	        
        } 
               
		if (subnumber > 0)
		{
		    PAL::SetAPIMode(g_emode);
			ros::WallTime t1 = ros::WallTime::now();
			overlaid1 = (left1Subnumber && laserscan1Subnumber);       

            data1 = PAL::GrabRangeScanData();
      		updatePC = true;
                       
			ros::WallTime t2 = ros::WallTime::now();						
			//ROS_INFO_STREAM("Grab time (ms): " << (t2 - t1).toNSec()*1e-6);					
		}
		
		

		ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();
	}
	
	PAL::Destroy();
}

