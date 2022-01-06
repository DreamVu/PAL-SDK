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

using namespace cv;
using namespace std;

using namespace std::chrono;

namespace PAL
{

	int RunTrack(cv::Mat& img, cv::Mat& depth, vector<vector<float>> &boxes, 
	    vector<int> &ids, vector<float> &depthValues, vector<Scalar> &colours);
}

int main(int argc, char *argv[])
{
   	
	namedWindow("Pal Object Tracking", WINDOW_NORMAL); // Create a window for display.

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
	prop.vertical_flip =0;
	PAL::SetCameraProperties(&prop, &flag);

	bool isDisparityNormalized = true;

	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	resizeWindow("Pal Object Tracking", width / 4, (height / 4) * 3);

	int key = ' ';

	printf("Press ESC to close the window\n");
	printf("Press v/V key to toggle the vertical flip of panorama\n");
	printf("Press f/F to toggle filter rgb property.\n");
	printf("Press d/D to toggle fast depth property\n");
	printf("Press r/R to toggle near range property\n");
	printf("Press s/S to toggle depth calculation functionality\n");

	size_t currentResolution = 0;

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

	//27 = esc key. Run the loop until the ESC key is pressed
	while (key != 27)
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

		for (int i = 0; i < boxes.size(); i++)
	    {
	    	if(useDepth)
	            putText(img, format("ID=%d , Depth=%.2fm", ids[i], depthValues[i]/100), Point(boxes[i][0], boxes[i][1] - 5), 
	            0, 0.6, Scalar(0, 0, 255), 1, LINE_AA);
	        else
	        	putText(img, format("ID=%d", ids[i]), Point(boxes[i][0], boxes[i][1] - 5), 
	            0, 0.6, Scalar(0, 0, 255), 1, LINE_AA);
            rectangle(img, Rect(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]), colours[i], 2);
	    }
	    putText(img, format("num: %d", num), Point(0, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

		//Display the final output image
		imshow("Pal Object Tracking", img);

		boxes.clear();
		ids.clear();
		if(useDepth)
			depthValues.clear();
		colours.clear();
		
		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;

		if (key == 's' || key == 'S')
		{
			useDepth = !useDepth;
		}
		if (key == 'v' || key == 'V')
		{
			PAL::CameraProperties prop;
			flip = !flip;
			prop.vertical_flip = flip;
			unsigned int flags = PAL::VERTICAL_FLIP;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 'f' || key == 'F')
		{	
			PAL::CameraProperties prop;
			filter_spots = !filter_spots;
			prop.filter_spots = filter_spots;
			unsigned int flags = PAL::FILTER_SPOTS;
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
	}
	printf("exiting the application\n");
	//CloseTrack();
	PAL::Destroy();

	return 0;
}
