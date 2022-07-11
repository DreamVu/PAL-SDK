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

#include "TimeLogger.h"
#include <fstream>
#include<boost/algorithm/string.hpp>


using namespace std;
using namespace cv;
//using namespace PAL;

namespace PAL
{
	namespace Internal
	{

		void EnableDepth(bool flag);
		void MinimiseCompute(bool flag);

	}
	int RunTrack(cv::Mat& img, cv::Mat& depth, vector<vector<float>> &boxes, 
	    vector<int> &ids, vector<float> &depthValues, vector<Scalar> &colours);
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
image_transport::Publisher rightpub;
image_transport::Publisher depthpub;
image_transport::Publisher personDet_pub;
image_transport::Publisher personDet3D_pub;
image_transport::Publisher socialDistance_pub;
image_transport::Publisher objectDet_pub;
image_transport::Publisher safeZoneDetection_pub;
image_transport::Publisher occupancyMap_pub;

image_transport::Publisher personTrack_pub;

ros::Publisher pointcloudPub;
ros::Publisher boxespub;


bool isDepthEnabled = true; 
vector<float> DepthValues;
vector<float> DepthValues2;



std::string PAL_nvidia_device;

std::string getDeviceName()
{
   	std::string file_name = "/proc/device-tree/nvidia,dtsfilename";
	std::string result;
	std::ifstream file(file_name);
	if(file.is_open())
	{
		while(file)
		{
			std::getline(file, result);
		}
	}
	file.close();

	std::vector<std::string> strs, strs2;
	boost::split(strs, result, boost::is_any_of("/"));
	boost::split(strs2, strs[strs.size()-1], boost::is_any_of("-"));
	
	std::string device_name;
	if (strs2[1] == "p3448")
	{
		device_name = "NANO";
	}
	else if (strs2[1] == "p3668")
	{
		device_name = "NX";
	}
	else if (strs2[1] == "p2888")
	{
		device_name = "AGX";
	}
	else if (strs2[1] == "p3701")
	{
		device_name = "AGX-ORIN";
	}
	else if (strs2[1] == "p3310")
	{
		device_name = "TX2";
	}
	else if (strs2[1] == "p3489")
	{
		device_name = "TX2-VARIANT";
	}
	else if (strs2[1] == "2180")
	{
		device_name = "TX1";
	}
	else
	{
		device_name="TK1";
	}
	return device_name;
}		



void setLabel(cv::Mat& input, const std::string label, const cv::Point org, cv::Scalar clr)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.8;
	int thickness = 2;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::rectangle(input, org + cv::Point(0, baseline), org + cv::Point(text.width, -text.height), CV_RGB(0,0,0), cv::FILLED);
	cv::putText(input, label, org, fontface, scale, clr, thickness, 4);
}

//Function to compute distance between two persons
bool IsSociallyDistant(PAL::Loc3D p1, PAL::Loc3D p2, int threshold)
{

	if((sqrt(pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0)+pow(p1.z-p2.z,2.0))) <= threshold)
		return false;
	return true;     
}

//Function to compute whether the detected persons are socially distant or not
void ComputeDistanceMatrix(std::vector<PAL::Loc3D> Loc3Ds, std::vector<bool>& DistantData, float threshold_distance)
{     
	int num_persons = Loc3Ds.size();
	bool b = true;

	for(int i=0; i<num_persons; i++)
	{
		for(int j = i+1; j<num_persons; j++)
		{     
			//checking if location of two persons are larger or not than 100cm
			b = IsSociallyDistant(Loc3Ds[i], Loc3Ds[j], threshold_distance);

			if(!b)
				DistantData[i] = DistantData[j] = false;
		}
	}
}    

cv::Mat Getoccupancy1D(cv::Mat rgb_image, cv::Mat depth_mat, int depth_thresh, float context_threshold) 
{
	cv::Mat mask;
	cv::threshold(depth_mat, mask, depth_thresh, 255, cv::THRESH_BINARY_INV);
	mask = mask/255;
	cv::Mat occupancySum = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_32FC1);

	cv::reduce(mask, occupancySum, 0, cv::REDUCE_SUM, CV_32FC1);

	cv::Mat occupancy1D = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_8UC1); //initialize with zeros;

	cv::threshold(occupancySum, occupancy1D, rgb_image.rows*context_threshold/100.0, 255, cv::THRESH_BINARY_INV);
	occupancy1D = occupancy1D/255;
	occupancy1D.convertTo(occupancy1D, CV_8UC1);		

	return occupancy1D;

}


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
	//cout << "LEFT : " << pLeft->cols << " " << pLeft->rows << endl;
	g_imgLeft = *pLeft;
	publishimage(g_imgLeft, leftpub, "bgr8");
}

void OnRightPanorama(cv::Mat* pRight)
{
	//cout << "RIGHT : " << pRight->cols << " " << pRight->rows << endl;
	g_imgRight = *pRight;
	publishimage(g_imgRight, rightpub, "bgr8");
}

void OnDepthPanorama(cv::Mat *img)
{
	//cout << "DEPTH  " << img->cols << " " << img->rows << endl;

	g_imgDepth = *img;

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


void OnPersonDetection(const std::vector<PAL::BoundingBox> Boxes, cv::Mat rgb)
{
	int num_of_persons = Boxes.size();
	char text[128];
	for(int i =0; i<num_of_persons; i++)
	{
		cv::rectangle(rgb,cv::Point(Boxes[i].x1, Boxes[i].y1), cv::Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,255,0), 2);
		if(isDepthEnabled)
		{
			sprintf(text, "Depth:%.2fm", DepthValues[i]/100);
			setLabel(rgb, text, Point(Boxes[i].x1, Boxes[i].y1), CV_RGB(0,255,0));
		}
	}
	//cout << "PERSON DETECTION  : " << rgb.cols << " " << rgb.rows << endl;
	publishimage(rgb, personDet_pub, "bgr8");

}

void OnPersonDetection3D(const std::vector<PAL::BoundingBox> Boxes, cv::Mat rgb, const vector<PAL::Loc3D> Loc3Ds)
{
	int num_of_persons = Boxes.size();
	char text[128];
	for(int i =0; i<num_of_persons; i++)
	{
		if(isDepthEnabled)
		sprintf(text, "x:%.1fm y:%.1fm z:%.1fm", Loc3Ds[i].x/100, Loc3Ds[i].y/100, Loc3Ds[i].z/100);

		if(DepthValues[i] > 3)
		{
		    cv::circle(rgb, Point((Boxes[i].x1+Boxes[i].x2)/2, (Boxes[i].y1+Boxes[i].y2)/2), 10, cv::Scalar(0,255,0), -1);
			if(isDepthEnabled)
		    setLabel(rgb, text, Point((Boxes[i].x1+Boxes[i].x2)/2, (Boxes[i].y1+Boxes[i].y2)/2-15), CV_RGB(0,255,0));
		}
		else
		{
		    cv::circle(rgb, Point((Boxes[i].x1+Boxes[i].x2)/2, (Boxes[i].y1+Boxes[i].y2)/2), 10, cv::Scalar(0,0,255), -1);
			if(isDepthEnabled)
		    setLabel(rgb, text, Point((Boxes[i].x1+Boxes[i].x2)/2, (Boxes[i].y1+Boxes[i].y2)/2-15), CV_RGB(255,0,0));
		}
	}
	//cout << "PERSON DETECTION  3D: " << rgb.cols << " " << rgb.rows << endl;
	publishimage(rgb, personDet3D_pub, "bgr8");

}

void OnSocialDistance(const std::vector<PAL::BoundingBox> Boxes, cv::Mat rgb, float threshold_distance, const vector<PAL::Loc3D> Loc3Ds)
{
	int num_of_persons = Boxes.size();
	std::vector<bool> DistantData(num_of_persons, true);
	char text[128];

	if(num_of_persons>=2)
	{
		ComputeDistanceMatrix(Loc3Ds, DistantData, threshold_distance);

		for(int i=0; i<num_of_persons; i++)
		{

			if(DistantData[i])
				//Drawing GREEN box indicating the person is socially distant
				cv::rectangle(rgb, cv::Point(Boxes[i].x1, Boxes[i].y1), cv::Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,255,0),2); 
			else
				//Drawing RED box indicating the person is not socially distant 
				cv::rectangle(rgb,Point(Boxes[i].x1, Boxes[i].y1), Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,0,255),2);
		}  
	}
	else if(num_of_persons==1)
	{
		cv::rectangle(rgb,Point(Boxes[0].x1, Boxes[0].y1), Point(Boxes[0].x2, Boxes[0].y2), cv::Scalar(0,255,0), 2);
	}

	//cout << "SOCIAL DISTANCE : " << rgb.cols << " " << rgb.rows << endl;
	publishimage(rgb, socialDistance_pub, "bgr8");

}

void OnObjectDetection(const std::vector<std::pair<int,PAL::BoundingBox>> Boxes, cv::Mat rgb)
{
	vector<string> classes = {"person","bicycle","car","motorcycle","4","bus","train","truck","8","traffic","light","fire hydrant","street sign","stop sign","parking meter","bench"};
	vector<vector<int>> color = {{100,200,0},{255,153,153},{250,0,0},{102,0,0},{0,0,0},{255,204,153},{255,128,0},{255,255,102},{0,0,0},{153,255,153},{102,102,255},{255,102,255},{255,0,127},{76,0,153},{64,64,64}, {126,126,126}};
	int num_of_persons = Boxes.size();
	char text[128];

	for(int i=0; i<num_of_persons; i++)
	{
		int k = Boxes[i].first;
		int p = (k*70)%255;
		if(isDepthEnabled)
		{
			sprintf(text, "%s Depth:%.2fm",classes[Boxes[i].first].c_str(), DepthValues2[i]/100);
		}
		else
		{
			sprintf(text, "%s",classes[Boxes[i].first].c_str());
		}

		cv::rectangle(rgb,cv::Point(Boxes[i].second.x1, Boxes[i].second.y1), cv::Point(Boxes[i].second.x2, Boxes[i].second.y2), CV_RGB(color[k][0],color[k][1],color[k][2]), 2);
		setLabel(rgb, text, Point(Boxes[i].second.x1, Boxes[i].second.y1),CV_RGB(color[k][0],color[k][1],color[k][2]));
	}

	//cout << "Object Detection : " << rgb.cols << " " << rgb.rows << endl;

	publishimage(rgb, objectDet_pub, "bgr8");
}

void OnSafeZoneDetection(const std::vector<PAL::BoundingBox> Boxes, cv::Mat rgb, const float safe_distance)
{
	int num_of_persons = Boxes.size();
	char text[128];
	for(int i =0; i<num_of_persons; i++)
	{
		if(isDepthEnabled)
		sprintf(text, "Depth:%.2fm", DepthValues[i]/100);

		if(DepthValues[i] > safe_distance)
		{
			cv::rectangle(rgb, cv::Point(Boxes[i].x1, Boxes[i].y1), cv::Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,255,0), 2);
			if(isDepthEnabled)
			setLabel(rgb, text, cv::Point(Boxes[i].x1, Boxes[i].y1), CV_RGB(0,255,0));
		}
		else
		{
			cv::rectangle(rgb, cv::Point(Boxes[i].x1, Boxes[i].y1), cv::Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,0,255), 2);
			if(isDepthEnabled)        
			setLabel(rgb, text, cv::Point(Boxes[i].x1, Boxes[i].y1), CV_RGB(255,0,0));
		}
	}

	//cout << "SAFE ZONE : " << rgb.cols << " " << rgb.rows << endl;

	publishimage(rgb, safeZoneDetection_pub, "bgr8");
}

void OnOccupancyMap(cv::Mat rgb, cv::Mat depth, const int threshold_cm, const int context_threshold)
{


	depth.convertTo(depth, CV_8UC1);

	cv::Mat occupancy1D = Getoccupancy1D(rgb, depth,threshold_cm,context_threshold);

	cv::Mat r = cv::Mat::ones(1, rgb.cols, CV_8UC1);
	cv::Mat g = cv::Mat::ones(1, rgb.cols, CV_8UC1);
	cv::Mat b = cv::Mat::ones(1, rgb.cols, CV_8UC1);

	unsigned char* poccupancy1D = (unsigned char* )occupancy1D.data;
	unsigned char* pr = (unsigned char* )r.data;

	for(int i=0; i<rgb.cols; i++,poccupancy1D++,pr++)
	{
		if((*poccupancy1D))
		{
			*pr = (*pr)*2;
		}

	}

	cv::Mat final_img;
	vector<Mat> channels;
	channels.push_back(b);
	channels.push_back(g);
	channels.push_back(r);

	merge(channels, final_img);
	resize(final_img, final_img, rgb.size());
	cv::Mat colored_out = rgb.mul(final_img);


	//cout << "OCCUPANCY: " << rgb.cols << " " << rgb.rows << endl;
	publishimage(colored_out, occupancyMap_pub, "bgr8");

}

void OnPointCloud(std::vector<PAL::Point> pc)
{
	sensor_msgs::PointCloud2Ptr pointcloudMsg;
	pointcloudMsg.reset(new sensor_msgs::PointCloud2);

	std::vector<PAL::Point>* point_data = &pc;

	pointcloudMsg->is_bigendian = false;
	pointcloudMsg->is_dense = false;

	sensor_msgs::PointCloud2Modifier modifier(*pointcloudMsg);
	pointcloudMsg->point_step = 4 * sizeof(float);

	pointcloudMsg->width = point_data->size();
	pointcloudMsg->height = 1;
	pointcloudMsg->row_step = sizeof(PAL::Point) * point_data->size();
	pointcloudMsg->header.frame_id = "pal";

	modifier.setPointCloud2Fields(4,
			"x", 1, sensor_msgs::PointField::FLOAT32,
			"y", 1, sensor_msgs::PointField::FLOAT32,
			"z", 1, sensor_msgs::PointField::FLOAT32,
			"rgb", 1, sensor_msgs::PointField::FLOAT32
			);


	PAL::Point* pointcloudPtr = (PAL::Point*)(&pointcloudMsg->data[0]);

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

	pointcloudPub.publish(pointcloudMsg);
}

void OnPersonTracking(Mat& rgb, vector<vector<float>> &boxes, 
    vector<int> &ids, vector<Scalar> &colours, int num)
{
	cv::Mat img = rgb.clone();
	for (int i = 0; i < boxes.size(); i++)
	{
	   putText(img, format("ID=%d", ids[i]), Point(boxes[i][0], boxes[i][1] - 5), 
	      0, 0.6, Scalar(0, 0, 255), 1, LINE_AA);
	   rectangle(img, Rect(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]), colours[i], 2);
	}
	putText(img, format("num: %d", num), Point(0, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

	boxes.clear();
	ids.clear();
	colours.clear();

	publishimage(img, personTrack_pub, "bgr8");
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "all_detection_node");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	//Creating all the publishers/subscribers
	leftpub               = it.advertise("/dreamvu/pal/persons/get/left", 1);
	rightpub              = it.advertise("/dreamvu/pal/persons/get/right", 1);
	depthpub              = it.advertise("/dreamvu/pal/persons/get/depth", 1);
	personDet_pub         = it.advertise("/dreamvu/pal/persons/get/person_detection", 1);
	personDet3D_pub       = it.advertise("/dreamvu/pal/persons/get/person_detection3D", 1);
	socialDistance_pub    = it.advertise("/dreamvu/pal/persons/get/social_distance", 1);
	objectDet_pub         = it.advertise("/dreamvu/pal/persons/get/object_detection", 1);  
	safeZoneDetection_pub = it.advertise("/dreamvu/pal/persons/get/safe_zone", 1);
	occupancyMap_pub      = it.advertise("/dreamvu/pal/persons/get/occupancy_map", 1);
	pointcloudPub         = nh.advertise<sensor_msgs::PointCloud2>("/dreamvu/pal/get/point_cloud", 1);
	personTrack_pub       = it.advertise("/dreamvu/pal/persons/get/person_track", 1);



	PAL_nvidia_device = getDeviceName();
	
	
	
	
	
	PAL::CameraProperties prop;
	PAL::CameraProperties prop_temp;
	PAL::GetCameraProperties(&prop);
	unsigned int flag = PAL::MODE;
	
	
	
	
	prop.mode = PAL::Mode::STEREO; // Change mode 
	
	
	
	
	prop_temp.mode = prop.mode;
	if((PAL_nvidia_device != "NANO" ) || (prop.mode == PAL::Mode::FAST_DEPTH)  || (prop.mode == PAL::Mode::HIGH_QUALITY_DEPTH)
	  || (prop.mode == PAL::Mode::POINT_CLOUD_25D) || (prop.mode == PAL::Mode::POINT_CLOUD_3D))
	{ 
		PAL::Internal::EnableDepth(isDepthEnabled);//isDepthEnabled);
		PAL::Internal::MinimiseCompute(false);
	}
	else
	{
		isDepthEnabled = false;
		PAL::Internal::EnableDepth(isDepthEnabled);//isDepthEnabled);
		PAL::Internal::MinimiseCompute(true);
	}
	


	//Initialising PAL

	

	int width, height;
	if(PAL::Init(width, height,-1) != PAL::SUCCESS) //Connect to the PAL camera
	{
		printf("Camera Init failed\n");
		return 1;
	}

	PAL::CameraProperties data; 
	PAL::Acknowledgement ack = PAL::LoadProperties("../catkin_ws/src/dreamvu_pal_camera/src/SavedPalProperties.txt", &data);
	if(ack != PAL::SUCCESS)
	{
		printf("Error Loading settings\n");
	}

	

	if (prop.mode==PAL::Mode::POINT_CLOUD_3D)
	{
		flag = flag | PAL::FD;
		prop.fd = 0;
	}
    else if(prop.mode == PAL::Mode::POINT_CLOUD_25D || prop.mode == PAL::Mode::FAST_DEPTH)
	{

	   flag = flag | PAL::FD;
		prop.fd = 1;
	}
	
	
	PAL::SetCameraProperties(&prop, &flag);

	//the nms threshold used to threshold the scores of the detected bounding boxes by the detection model.
	//Higher value means boxes with higher score will be displayed. Low value will increase false positives.
	float threshold =  0.35;

	//the threshold distance in cm. Used for checking whether social distance is maintained.
	//the threshold distance should be kept within 1m to 2m range
	float threshold_distance = 100.0f;

	//the safe distance in cm.
	float safe_distance = 200.0f;

	//depth threshold in cm. Used for generating occupancy map.
	//The depth threshold should be kept within 1m to 2m range.
	int threshold_cm = 100;
	
	//context threshold in percentage to be considered for occupancy
	//The context threshold should be kept within 50(recommended) to 80 range.
	int context_threshold = 50; 

	if(threshold_distance > 200)
	{
		threshold_distance = 200;
		printf("threshold distance set above maximum range. Setting to 2m\n");
	}
	else if(threshold_distance < 100)
	{
		threshold_distance = 100;
		printf("threshold distance set below minumum range. Setting to 1m\n");
	}

	if (threshold_cm > 200)
	{
		threshold_cm = 200;
		printf("depth threshold set above maximum range. Setting to 2m\n");
	}
	else if (threshold_cm < 100)
	{
		threshold_cm = 100;
		printf("depth threshold set below minimum range. Setting to 1m\n");
	}

	if (context_threshold > 80)
	{
		context_threshold = 80;
		printf("context threshold set above maximum range. Setting to 80%\n");
	}
	else if(context_threshold < 50)
	{
		context_threshold = 50;
		printf("context threshold set below miminum range. Setting to 50%\n");
	}


	if(PAL_nvidia_device != "NANO")
	{
		if(PAL::InitPersonDetection(threshold)!= PAL::SUCCESS) //Initialise person detection pipeline
		{
			printf("Person Detection Init failed\n");
			return 1;
		}
	}
	else 
	{
		if(prop.mode == PAL::Mode::DETECTION)
		{
			if(PAL::InitPersonDetection(threshold)!= PAL::SUCCESS) //Initialise person detection pipeline
			{
				printf("Person Detection Init failed\n");
				return 1;
			}
		}	
	}

	ros::Rate loop_rate(30);
	g_bRosOK = ros::ok();

	cv::Mat rgb, depth, output,right;
	cv::Mat rgb2, depth2, output2, right2;
	cv::Mat rgb3, depth3, right3;
	cv::Mat rgb4, rgb8;
	cv::Mat left5, right5;
	cv::Mat rgb6, right6, depth6;
	cv::Mat rgb7, right7, depth7;
	
	PAL::Image left_img, right_img, depth_img, disparity;
	PAL::Image left_img2, right_img2, depth_img2;
	PAL::Image g_imgLeft, g_imgRight, g_imgDepth;
	PAL::Image left_img3, right_img3, depth_img3;
	std::vector<PAL::BoundingBox> Boxes;
	std::vector<std::pair<int,PAL::BoundingBox>> Boxes2;  
	vector<PAL::Loc3D> Loc3Ds;
	std::vector<PAL::Point> pc;
	vector<vector<float>> boxes3; 
	vector<int> ids; 
	vector<float> depthValues3; 
	vector<Scalar> colours; 
	int num;

	cv::Mat left_img_S = cv::Mat::zeros(1819, 5290, CV_8UC3);
	cv::Mat right_img_S = cv::Mat::zeros(1819, 5290, CV_8UC3);
	
	while(g_bRosOK)
	{
		//Getting no of subscribers for each publisher
		int leftSubnumber = leftpub.getNumSubscribers();
		int rightSubnumber = rightpub.getNumSubscribers();
		int depthSubnumber = depthpub.getNumSubscribers();
		int personDet_number = personDet_pub.getNumSubscribers();
		int personDet3D_number = personDet3D_pub.getNumSubscribers();
		int socialDistance_number = socialDistance_pub.getNumSubscribers();
		int objectDet_number = objectDet_pub.getNumSubscribers();
		int safeZoneDetection_number = safeZoneDetection_pub.getNumSubscribers();
		int occupancyMap_number = occupancyMap_pub.getNumSubscribers();
		int pointcloudSubnumber = pointcloudPub.getNumSubscribers();
		int personTrack_number = personTrack_pub.getNumSubscribers();

		int detection_Subnumber = leftSubnumber+rightSubnumber+depthSubnumber+personDet_number+safeZoneDetection_number+personDet3D_number+socialDistance_number;
		int objectdetection_Subnumber = leftSubnumber+rightSubnumber+depthSubnumber+objectDet_number;
		int fastdepth_Subnumber  = leftSubnumber+rightSubnumber+depthSubnumber+occupancyMap_number;
		int stereo_Subnumber    = leftSubnumber+rightSubnumber;
      	int pointcloud25dSubnumber = leftSubnumber+rightSubnumber+pointcloudSubnumber;
     	int track_Subnumber = leftSubnumber+rightSubnumber+depthSubnumber+personTrack_number;
        
		DepthValues.clear();
		DepthValues2.clear();
		Boxes.clear();
		Boxes2.clear();
		Loc3Ds.clear();
		pc.clear();
		
		if(stereo_Subnumber && (prop_temp.mode==PAL::Mode::STEREO))
		{
			
			
			timeval timestamp;
			cv::Mat stereo_output = PAL::GetCroppedStereo(5290, 3638, 0, 0, timestamp,1);
		
			
			if (leftSubnumber > 0) 
			{
				memcpy(left_img_S.data, stereo_output.data , left_img_S.step*left_img_S.rows);
				OnLeftPanorama(&left_img_S);
				
			}
			if (rightSubnumber > 0) 
			{
				memcpy(right_img_S.data, stereo_output.data + left_img_S.step*left_img_S.rows, left_img_S.step*left_img_S.rows);
				OnRightPanorama(&right_img_S);
				
			}	
		}

		if(detection_Subnumber && (prop_temp.mode==PAL::Mode::DETECTION))
		{
			PAL::Acknowledgement ack2 = PAL::GetPeopleDetection(rgb, right, depth, &Boxes, DepthValues, &Loc3Ds, NULL);
#if 1			
			if(!isDepthEnabled)
			{
				Loc3Ds.clear();
				Loc3Ds.resize(Boxes.size());
				for(int i=0 ; i< Boxes.size(); i++)
				{
					Loc3Ds[i].x = -1;
					Loc3Ds[i].y = -1;
					Loc3Ds[i].z = -1;		
				}
			}	
#endif			
			rgb4 = rgb.clone();
			if (personDet_number > 0) OnPersonDetection(Boxes, rgb4);
			rgb4 = rgb.clone();
			if (personDet3D_number > 0) OnPersonDetection3D(Boxes, rgb4, Loc3Ds);
			rgb4 = rgb.clone();
			if (socialDistance_number > 0) OnSocialDistance(Boxes, rgb4, threshold_distance, Loc3Ds);
			rgb4 = rgb.clone();
			if (safeZoneDetection_number > 0) OnSafeZoneDetection(Boxes, rgb4, safe_distance);

			if (leftSubnumber > 0) OnLeftPanorama(&rgb);
			if (rightSubnumber > 0) OnRightPanorama(&right);
			if (depthSubnumber > 0) OnDepthPanorama(&depth);
		}

		if(objectdetection_Subnumber  && (prop_temp.mode==PAL::Mode::DETECTION))
		{ 
			PAL::Acknowledgement ack3 = PAL::GetAllDetection(rgb2, right2, depth2, &Boxes2, DepthValues2);
			rgb8 = rgb.clone();
			if (objectDet_number > 0) OnObjectDetection(Boxes2, rgb2); 

			if (leftSubnumber > 0) OnLeftPanorama(&rgb8);
			if (rightSubnumber > 0) OnRightPanorama(&right2);
			if (depthSubnumber > 0) OnDepthPanorama(&depth2);
		}

		if(fastdepth_Subnumber && ((prop_temp.mode==PAL::Mode::FAST_DEPTH) || (prop_temp.mode==PAL::Mode::HIGH_QUALITY_DEPTH)))
		{
			PAL::Acknowledgement ack3 = PAL::GrabFrames(&left_img, &right_img, &depth_img);
			depth3 = cv::Mat(depth_img.rows, depth_img.cols, CV_32FC1, depth_img.Raw.f32_data);
			rgb3 = cv::Mat(left_img.rows, left_img.cols, CV_8UC3, left_img.Raw.u8_data);
			right3 = cv::Mat(right_img.rows, right_img.cols, CV_8UC3, right_img.Raw.u8_data);
			

			if (leftSubnumber > 0) OnLeftPanorama(&rgb3);
			if (rightSubnumber > 0) OnRightPanorama(&right3);
			if (depthSubnumber > 0) OnDepthPanorama(&depth3);
		}
		
		if(pointcloudSubnumber && (prop_temp.mode==PAL::Mode::POINT_CLOUD_3D))
		{
			PAL::Acknowledgement ack = PAL::GetPointCloud(&pc, 0, &g_imgLeft, &g_imgRight, &g_imgDepth);
			OnPointCloud(pc);
		}
		
		if((pointcloud25dSubnumber && (prop_temp.mode==PAL::Mode::POINT_CLOUD_25D)) || occupancyMap_number)
		{
			PAL::Acknowledgement ack = PAL::GetPointCloud(&pc, 0, &g_imgLeft, &g_imgRight, &g_imgDepth);
			
			depth6 = cv::Mat(g_imgDepth.rows, g_imgDepth.cols, CV_32FC1, g_imgDepth.Raw.f32_data);
			rgb6 = cv::Mat(g_imgLeft.rows, g_imgLeft.cols, CV_8UC3, g_imgLeft.Raw.u8_data);
			right6 = cv::Mat(g_imgRight.rows, g_imgRight.cols, CV_8UC3, g_imgRight.Raw.u8_data);
			
			cv::resize(depth6, depth6, cv::Size(rgb6.cols, rgb6.rows));
			
			if (occupancyMap_number > 0) OnOccupancyMap(rgb6, depth6, threshold_cm, context_threshold);


			if (leftSubnumber > 0) OnLeftPanorama(&rgb6);
			if (rightSubnumber > 0) OnRightPanorama(&right6);
			if (depthSubnumber > 0) OnDepthPanorama(&depth6);
			if (pointcloudSubnumber > 0) OnPointCloud(pc);
		}

		if(track_Subnumber  && (prop_temp.mode==PAL::Mode::TRACKING))
		{
			PAL::Acknowledgement ack = PAL::GrabFrames(&left_img3, &right_img3); 
			rgb7 = Mat(left_img3.rows, left_img3.cols, CV_8UC3, left_img3.Raw.u8_data);
			right7 = Mat(right_img3.rows, right_img3.cols, CV_8UC3, right_img3.Raw.u8_data);
			if (personTrack_number > 0) 
			{
				depth7 = cv::Mat::zeros(cv::Size(1, 1), CV_32FC1);
				num = PAL::RunTrack(rgb7, depth7, boxes3, ids, depthValues3, colours);
				OnPersonTracking(rgb7, boxes3, ids, colours, num); 
			}

			if (leftSubnumber > 0) OnLeftPanorama(&rgb7);
			if (rightSubnumber > 0) OnRightPanorama(&right7);
		}

		//ros::spinOnce();
		loop_rate.sleep();
		g_bRosOK = ros::ok();		
	}

	return 0;
}
