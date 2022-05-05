/*

   CODE SAMPLE # 009: Object Detection
   This code sample allows users to run Object Detection and find the Depth of each detected object. 


   >>>>>> Compile this code using the following command....

g++ 009_object_detection.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DE.so ../lib/libPAL_EDET.so `pkg-config --libs --cflags opencv`   -O3  -o 009_object_detection.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl  -lX11


   >>>>>> Execute the binary file by typing the following command...

   ./009_object_detection.out


   >>>>>> KEYBOARD CONTROLS:
	
	Press ESC to close the window
	Press q/Q to toggle Depth detection mode
	Press s/S to toggle 3D location detection mode
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
# include <chrono>
# include <stdio.h>
# include <opencv2/opencv.hpp>
# include "PAL.h"
#include <X11/Xlib.h>
using namespace cv;
using namespace std;
using namespace std::chrono;

namespace PAL
{
	namespace Internal
	{
		void EnableDepth(bool flag);
		void MinimiseCompute(bool flag);
	}
}

void setLabel(cv::Mat& input, const std::string label, const cv::Point org, cv::Scalar clr)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.5;
	int thickness = 2;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::rectangle(input, org + cv::Point(0, baseline), org + cv::Point(text.width, - text.height), CV_RGB(0,0,0), cv::FILLED);
	cv::putText(input, label, org, fontface, scale, clr, thickness, 4);
}

int main(int argc, char *argv[])
{
	// Create a window for display.
	namedWindow( "PAL Object Detection", WINDOW_NORMAL ); 

	//Enable/Disable depth alongside person detection	
	bool isDepthEnabled = true;  
	PAL::Internal::EnableDepth(isDepthEnabled);
	PAL::Internal::MinimiseCompute(false);

	int width, height;
	//Connect to the PAL camera
	if(PAL::Init(width, height,-1) != PAL::SUCCESS) 
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

	//If a command line argument is passed then that will be taken as threshold
	float threshold = (argc>1) ? atof(argv[1]) : 0.35;

	if(PAL::InitPersonDetection(threshold)!= PAL::SUCCESS) //Initialise object detection pipeline
	{
		printf("Object Detection Init failed\n");
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
	
	resizeWindow( "PAL Object Detection", sc_width, sc_height);//width/4, (height/4)*2);

	int key = ' '; 

	printf("Press ESC to close the window\n");    
	printf("Press q/Q to toggle Depth detection mode\n");
	printf("Press s/S to toggle 3D location detection mode\n");
	printf("Press f/F to toggle filter rgb property\n");
	printf("Press d/D to toggle fast depth property\n");
	printf("Press v/V to toggle vertical flip property\n");
	printf("Press r/R to toggle near range property\n");
	printf("Press l/L to switch floor mode\n");
	printf("Press i/I to switch intermediate mode\n");
	printf("Press t/T to switch table top mode\n");
	printf("Press c/C to switch ceiling mode\n");
	printf("Press a/A to switch auto mode\n");  


	bool filter_spots = true;	
	bool flip = false;
	bool fd = true;
	bool nr = false;
	bool loc3dIsEnabled = true;
	//27 = esc key. Run the loop until the ESC key is pressed
	int k=0;
	vector<string> classes = {"person","bicycle","car","motorcycle","","bus","train","truck","","traffic","light","fire hydrant","street sign","stop sign","parking meter","bench"};
	vector<vector<int>> color = {{100,200,0},{255,153,153},{250,0,0},{102,0,0},{0,0,0},{255,204,153},{255,128,0},{255,255,102},{0,0,0},{153,255,153},{102,102,255},{255,102,255},{255,0,127},{76,0,153},{64,64,64}, {126,126,126}};
	while(key != 27)
	{		 	
		auto start = high_resolution_clock::now();

		cv::Mat rgb, depth, output,right;
		std::vector<std::pair<int,PAL::BoundingBox>> Boxes;  
		vector<float> DepthValues;
		vector<PAL::Loc3D> Loc3Ds;
		if(loc3dIsEnabled && isDepthEnabled){
			PAL::GetAllDetection(rgb,right, depth, &Boxes, DepthValues, &Loc3Ds, NULL);
		}
		else{
			PAL::GetAllDetection(rgb,right, depth, &Boxes, DepthValues);
		}
		int num_of_objects = Boxes.size();
		char text[128];
		for(int i =0; i<num_of_objects; i++)
		{
			int k = Boxes[i].first;

			int p = (k*70)%255;
			if(isDepthEnabled){
				if(loc3dIsEnabled)
				{
					sprintf(text, "%s x:%.1fm y:%.1fm z:%.1fm", classes[Boxes[i].first].c_str(), Loc3Ds[i].x/100, Loc3Ds[i].y/100, Loc3Ds[i].z/100);
					cv::circle(rgb, Point((Boxes[i].second.x1+Boxes[i].second.x2)/2, (Boxes[i].second.y1+Boxes[i].second.y2)/2), 5, CV_RGB(color[k][0],color[k][1],color[k][2]), -1);
					setLabel(rgb, text, Point((Boxes[i].second.x1+Boxes[i].second.x2)/2, (Boxes[i].second.y1+Boxes[i].second.y2)/2-10), CV_RGB(color[k][0],color[k][1],color[k][2]));
				}
				else
				{
					sprintf(text, "%s Depth:%.2fm", classes[Boxes[i].first].c_str(), DepthValues[i]/100);
					cv::rectangle(rgb,Point(Boxes[i].second.x1, Boxes[i].second.y1), Point(Boxes[i].second.x2, Boxes[i].second.y2), CV_RGB(color[k][0],color[k][1],color[k][2]), 2);
					setLabel(rgb, text, Point(Boxes[i].second.x1, Boxes[i].second.y1), CV_RGB(color[k][0],color[k][1],color[k][2]));
				}
			}
			else
			{
				sprintf(text, "%s",classes[Boxes[i].first].c_str());
				cv::rectangle(rgb,Point(Boxes[i].second.x1, Boxes[i].second.y1), Point(Boxes[i].second.x2, Boxes[i].second.y2), CV_RGB(color[k][0],color[k][1],color[k][2]), 2);
				setLabel(rgb, text, Point(Boxes[i].second.x1, Boxes[i].second.y1), CV_RGB(color[k][0],color[k][1],color[k][2]));
			}
		}

		imshow( "PAL Object Detection", rgb); 

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;
		if (key == 'q' || key == 'Q')
		{
			isDepthEnabled = !isDepthEnabled;
			PAL::Internal::EnableDepth(isDepthEnabled);
		}
		if (key == 's' || key == 'S')
		{
			loc3dIsEnabled = !loc3dIsEnabled;
		}
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
