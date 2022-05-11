/*

   CODE SAMPLE # 008: Social Distancing
   This code sample allows users to check if the distance between two people is within a limit or not.


   >>>>>> Compile this code using the following command....

g++  008_social_distancing.cpp  /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DE.so ../lib/libPAL_EDET.so `pkg-config --libs --cflags opencv`   -O3  -o 008_social_distancing.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl -lstdc++fs -lX11


   >>>>>> Execute the binary file by typing the following command...

   ./008_social_distancing.out


	>>>>>> KEYBOARD CONTROLS:
		Press ESC to close the window
		Press f/F to toggle filter rgb property
		Press d/D to toggle fast depth property
		Press v/V to toggle vertical flip property
		Press r/R to toggle near range property
		Press l/L to switch floor mode
		Press i/I to switch intermediate mode
		Press t/T to switch table top mode
		Press c/C to switch ceiling mode
		Press a/A to switch auto mode

*/
# include <stdio.h>
# include <opencv2/opencv.hpp>
# include "PAL.h"
#include <X11/Xlib.h>
using namespace cv;
using namespace std;

namespace PAL
{
	namespace Internal
	{
		void EnableDepth(bool flag);
		void MinimiseCompute(bool flag);
	}
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

int main( int argc, char** argv )
{
	namedWindow( "PAL Social Distancing", WINDOW_NORMAL ); // Create a window for display.

	//Depth should be enabled for this code sample as a prerequisite    
	bool isDepthEnabled = true; 
	PAL::Internal::EnableDepth(isDepthEnabled);
	PAL::Internal::MinimiseCompute(false);

	int width, height;
	if(PAL::Init(width, height,-1) != PAL::SUCCESS) //Connect to the PAL camera
	{
		printf("Camera Init failed\n");
		return 1;
	}

	PAL::CameraProperties data; 
	PAL::Acknowledgement ack = PAL::LoadProperties("../Explorer/SavedPalProperties.txt");
	if(ack != PAL::SUCCESS)
	{
		printf("Error Loading settings\n");
	}

	PAL::CameraProperties prop;

	unsigned int flag = PAL::MODE;
	flag = flag | PAL::FD;
	flag = flag | PAL::NR;
	flag = flag | PAL::FILTER_SPOTS;
	flag = flag | PAL::VERTICAL_FLIP;
	
	prop.mode = PAL::Mode::DETECTION;
	prop.fd = 1;
	prop.nr = 0;
	prop.filter_spots = 1;
	prop.vertical_flip =0;
	PAL::SetCameraProperties(&prop, &flag);

	float threshold = (argc>1) ? atof(argv[1]) : 0.35;
	float threshold_distance = (argc>2) ? atof(argv[2]) : 100.0f; //threshold_distance should be between 1m to 2m.

	if(threshold_distance > 200)
	{
		threshold_distance = 200;
		printf("threshold distance set above maximum range. Setting to 2m");
	}
	else if(threshold_distance < 100)
	{
		threshold_distance = 100;
		printf("threshold distance set below minumum range. Setting to 1m");
	}

	if(PAL::InitPersonDetection(threshold)!= PAL::SUCCESS) //Initialise object detection pipeline
	{
		printf("Social Distancing Init failed\n");
		return 1;
	}

	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	// Getting Screen resolution 
	Display* disp = XOpenDisplay(NULL);
	Screen*  scrn = DefaultScreenOfDisplay(disp);
	int sc_height = scrn->height;
	int sc_width  = scrn->width;
	
	resizeWindow("PAL Social Distancing", sc_width, sc_height);//width/4, (height/4)*2);

	int key = ' ';

	printf("Press ESC to close the window\n");    
	printf("Press f/F to toggle filter rgb property\n");
	printf("Press d/D to toggle fast depth property\n");
	printf("Press v/V to toggle vertical flip property\n");
	printf("Press r/R to toggle near range property\n");
	printf("Press l/L to switch floor mode\n");
	printf("Press i/I to switch intermediate mode\n");
	printf("Press t/T to switch table top mode\n");
	printf("Press c/C to switch ceiling mode\n");
	printf("Press a/A to switch auto mode\n");

	bool flip = false;
	bool fd = true;
	bool nr = false;
	bool filter_spots = true;

	//27 = esc key. Run the loop until the ESC key is pressed

	while(key != 27)
	{

		cv::Mat rgb, depth, output,right;
		std::vector<PAL::BoundingBox> Boxes;  
		vector<float> DepthValues;
		vector<PAL::Loc3D> Loc3Ds;
		//Function to Query 
		//Image data: rgb & depth panoramas 
		//Person detection data: Bounding boxes and 3-D locations of each person detected.    
		PAL::GetPeopleDetection(rgb,right, depth, &Boxes, DepthValues, &Loc3Ds);	

		int num_of_persons = Boxes.size();
		std::vector<bool> DistantData(num_of_persons, true);

		//Computing if persons are socially distant or not in case of multiple detections using 3-D locations
		if(num_of_persons>=2)
		{
			ComputeDistanceMatrix(Loc3Ds, DistantData, threshold_distance);
			for(int i=0; i<num_of_persons; i++)
			{

				if(DistantData[i])
					cv::rectangle(rgb,Point(Boxes[i].x1, Boxes[i].y1), Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,255,0),2); //Drawing GREEN box indicating the person is socially distant
				else
					cv::rectangle(rgb,Point(Boxes[i].x1, Boxes[i].y1), Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,0,255),2); //Drawing RED box indicating the person is not socially distant 
			}  
		}
		else if(num_of_persons==1)
		{
			cv::rectangle(rgb,Point(Boxes[0].x1, Boxes[0].y1), Point(Boxes[0].x2, Boxes[0].y2), cv::Scalar(0,255,0), 2);
		}

		//Display the final output image
		imshow( "PAL Social Distancing", rgb); 

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;

		if (key == 'f' || key == 'F')
		{   
			PAL::CameraProperties prop;
			filter_spots = !filter_spots;
			prop.filter_spots = filter_spots; 
			unsigned int flags = PAL::FILTER_SPOTS;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 'v' || key == 'V')
		{           
			PAL::CameraProperties prop;
			flip = !flip;
			prop.vertical_flip = flip; 
			unsigned int flags = PAL::VERTICAL_FLIP;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 'l' || key == 'L')
		{	
			PAL::CameraProperties prop;
			prop.mode = PAL::Mode::DETECTION;
			prop.detection_mode = PAL::FLOOR;  
			unsigned int flags = PAL::DETECTION_MODE | PAL::MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 't' || key == 'T')
		{	
			PAL::CameraProperties prop;
			prop.mode = PAL::Mode::DETECTION;
			prop.detection_mode = PAL::TABLE_TOP; 
			unsigned int flags = PAL::DETECTION_MODE | PAL::MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 'c' || key == 'C')
		{	
			PAL::CameraProperties prop;
			prop.mode = PAL::Mode::DETECTION;
			prop.detection_mode = PAL::CEILING; 
			unsigned int flags = PAL::DETECTION_MODE | PAL::MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 'i' || key == 'I')
		{	
			PAL::CameraProperties prop;
			prop.mode = PAL::Mode::DETECTION;
			prop.detection_mode = PAL::INTERMEDIATE; 
			unsigned int flags = PAL::DETECTION_MODE | PAL::MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 'a' || key == 'A')
		{	
			PAL::CameraProperties prop;
			prop.mode = PAL::Mode::DETECTION;
			prop.detection_mode = PAL::AUTO; 
			unsigned int flags = PAL::DETECTION_MODE | PAL::MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if(key == 'd' || key == 'D')
		{
			PAL::CameraProperties prop;
			fd = !fd;
			prop.fd = fd;
			unsigned int flags = PAL::FD; 
			PAL::SetCameraProperties(&prop, &flags);
		}
		if(key == 'r' || key == 'R')
		{       
			PAL::CameraProperties prop;
			nr = !nr;
			prop.nr = nr;
			unsigned int flags = PAL::NR;
			PAL::SetCameraProperties(&prop, &flags);
		}  
		PAL::CameraProperties properties;
		GetCameraProperties(&properties);
		filter_spots = properties.filter_spots;	
		flip = properties.vertical_flip;
		fd = properties.fd;
		nr = properties.nr;      
	}

	printf("exiting the application\n");
	PAL::Destroy();

	return 0;
}
