/*

CODE SAMPLE # 011: Object Tracking
This code will grab the basic stereo panoramas (left and right images) and ALSO the Disparity panorama, execute tracking for objects on it and then display in an opencv window


>>>>>> Compile this code using the following command....


g++ 011_object_tracking.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DE.so libPAL_Track.so `pkg-config --libs --cflags opencv`   -g  -o 011_object_tracking.out -I../include/ -I/usr/local/include/eigen3     -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl


>>>>>> Execute the binary file by typing the following command...


./011_object_tracking.out


>>>>>> KEYBOARD CONTROLS:

	   ESC key closes the window
	   	Press v/V key to toggle the vertical flip of panorama
		Press f/F to toggle filter rgb property.
		Press d/D to toggle fast depth property
		Press r/R to toggle near range property

*/


# include <stdio.h>
# include <opencv2/opencv.hpp>
# include <chrono>
# include <bits/stdc++.h>
# include "PAL.h"
#include <JetsonGPIO/JetsonGPIO.h>

static bool g_bExit = false;

void signalHandler( int signum )
{
	g_bExit = true;
}


//camera and detection pins
int camera_pin = 15;
int detection_pin = 7;
int camera_pin_1 = 29;
int camera_pin_2 = 31;
int camera_pin_3 = 33;

using namespace cv;
using namespace std;

using namespace std::chrono;

namespace PAL
{

	int RunTrack(cv::Mat& img, cv::Mat& depth, vector<vector<float>> &boxes, 
	    vector<int> &ids, vector<float> &depthValues, vector<Scalar> &colours);
}

namespace PAL
{
	namespace Internal
	{
		void EnableDepth(bool flag);
		void MinimiseCompute(bool flag);
	}
}

int main(int argc, char *argv[])
{
   	
	signal(SIGINT, signalHandler);

	PAL::Internal::EnableDepth(false);
	PAL::Internal::MinimiseCompute(true);

	//setting mode to GPIO board
	GPIO::setmode(GPIO::BOARD);

	//setting camera pins active
	GPIO::setup(camera_pin, GPIO::OUT, GPIO::LOW);
	GPIO::setup(camera_pin_1, GPIO::OUT, GPIO::LOW);
	GPIO::setup(camera_pin_2, GPIO::OUT, GPIO::LOW);
	GPIO::setup(camera_pin_3, GPIO::OUT, GPIO::LOW); 

	//setting detection pin inactive by default
	GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);

	int width, height;
	if (PAL::Init(width, height, -1) != PAL::SUCCESS) //Connect to the PAL camera
	{
		printf("Init failed\n");
		return 1;
	}

	PAL::CameraProperties data; 
	PAL::Acknowledgement ack = PAL::LoadProperties("../Explorer/SavedPalProperties.txt", &data);
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

	prop.mode = PAL::Mode::TRACKING;
	prop.fd = 1;
	prop.nr = 0;
	prop.filter_spots = 1;
	prop.vertical_flip = 0;
	PAL::SetCameraProperties(&prop, &flag);

	printf("Press Ctrl+C to exit the app\n");   


	bool flip = false;
	bool filter_spots = true;
	bool nr = false;
	bool fd = true;

	vector<vector<float>> boxes; 
	vector<int> ids; 
	vector<float> depthValues; 
	vector<Scalar> colours; 
	int num;
	bool useDepth = false;

	bool bDetectionPinActive = false;

	//27 = esc key. Run the loop until the ESC key is pressed
	while(!g_bExit)
	{
		PAL::Image left, right, depth, disparity;
		Mat img, d;
		if (useDepth)
			PAL::GrabFrames(&left, &right, &depth);
		else
			PAL::GrabFrames(&left, &right);
		
		//Convert PAL::Image to Mat
		img = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
		if (useDepth)
		{
			d = Mat(depth.rows, depth.cols, CV_32FC1, depth.Raw.f32_data);
		}
		else
		{
			d = cv::Mat::zeros(cv::Size(1, 1), CV_32FC1);
		}
		
		num = PAL::RunTrack(img, d, boxes, ids, depthValues, colours);

		if(num)
		{
			if(!bDetectionPinActive)
			{
				cout<<"Setting GPIO detection pin active"<<endl;
				GPIO::setup(detection_pin, GPIO::OUT, GPIO::LOW);

				bDetectionPinActive = true;
			}

		}
		else
		{
			if(bDetectionPinActive)
			{
				cout<<"Setting GPIO detection pin inactive"<<endl;
				GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);

				bDetectionPinActive = false;
			}

		}

		boxes.clear();
		ids.clear();
		if(useDepth)
			depthValues.clear();
		colours.clear();
		
		
	}

	printf("exiting the application\n");

	GPIO::setup(camera_pin, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(camera_pin_1, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(camera_pin_2, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(camera_pin_3, GPIO::OUT, GPIO::HIGH);

	GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);

	GPIO::cleanup();

	PAL::Destroy();

	return 0;
}
