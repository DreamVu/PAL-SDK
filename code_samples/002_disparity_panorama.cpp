/*

CODE SAMPLE # 002: Disparity panorama
This code will grab the basic stereo panoramas (left and right images) and ALSO the Disparity panorama, and all these 3 images are displayed in an opencv window


>>>>>> Compile this code using the following command....


g++ 002_disparity_panorama.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`    -g  -o 002_disparity_panorama.out -I../include/ -lv4l2 -lpthread -std=c++11


>>>>>> Execute the binary file by typing the following command...


./002_disparity_panorama.out


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


int main(int argc, char *argv[])
{
   	
	namedWindow("PAL Disparity Panorama", WINDOW_NORMAL); // Create a window for display.

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

	prop.mode = PAL::Mode::FAST_DEPTH; // The other available option is PAL::Mode::HIGH_QUALITY_DEPTH
	prop.fd = 1;
	prop.nr = 0;
	prop.filter_spots = 1;
	prop.vertical_flip =0;
	PAL::SetCameraProperties(&prop, &flag);

	bool isDisparityNormalized = true;


	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	resizeWindow("PAL Disparity Panorama", width / 4, (height / 4) * 3);

	int key = ' ';

	printf("Press ESC to close the window\n");
	printf("Press v/V key to toggle the vertical flip of panorama\n");
	printf("Press f/F to toggle filter rgb property.\n");
	printf("Press d/D to toggle fast depth property\n");
	printf("Press r/R to toggle near range property\n");

	size_t currentResolution = 0;

	bool flip = false;
	bool filter_spots = true;
	bool nr = false;
	bool fd = true;

	//27 = esc key. Run the loop until the ESC key is pressed
	while (key != 27)
	{
		PAL::Image left, right, depth, disparity;
		PAL::GrabFrames(&left, &right, &depth);

		Mat output;
		//Convert PAL::Image to Mat
		Mat l = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
		Mat r = Mat(right.rows, right.cols, CV_8UC3, right.Raw.u8_data);
		Mat d = Mat(depth.rows, depth.cols, CV_32FC1, depth.Raw.f32_data);
		d.convertTo(d, CV_8UC1);
		cvtColor(d, d, cv::COLOR_GRAY2BGR);

		//Vertical concatenation of temp and disparity into the final output
		vconcat(l, d, output);

		//Display the final output image
		imshow("PAL Disparity Panorama", output);

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;

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
	PAL::Destroy();

	return 0;
}
