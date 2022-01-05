/*

CODE SAMPLE # 002: Disparity panorama
This code will grab the basic stereo panoramas (left and right images) and ALSO the Disparity panorama, and all these 3 images are displayed in an opencv window


>>>>>> Compile this code using the following command....


g++ 002_disparity_panorama.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`    -g  -o 002_disparity_panorama.out -I../include/ -lv4l2 -lpthread -std=c++11


>>>>>> Execute the binary file by typing the following command...


./002_disparity_panorama.out


>>>>>> KEYBOARD CONTROLS:

	   ESC key closes the window
	   'N' key toggles the disparity normalization
	   'F' key toggles the vertical flip of panorama
	   'Y' key toggles the colorspace : RGB or YUV444

*/




# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"

using namespace cv;
using namespace std;


int main(int argc, char *argv[])
{
   	namedWindow("PAL window", WINDOW_NORMAL); // Create a window for display.

	int width, height;
	if (PAL::Init(width, height, -1) != PAL::SUCCESS) //Connect to the PAL camera
	{
		printf("Init failed\n");
		return 1;
	}

	bool isDisparityNormalized = true;

	printf("The image resolution is .... %dx%d\n", width, height);


	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	resizeWindow("PAL window", width / 4, (height / 4) * 3);

	int key = ' ';

	printf("Press ESC to close the window\n");
	printf("Press N to toggle the disparity normalization\n");
	printf("Press v/V key to toggle the vertical flip of panorama\n");
	printf("Press f/F to toggle filter rgb property.\n");

	size_t currentResolution = 0;

	bool flip = false;
	bool filter_spots = true;

	PAL::CameraProperties prop;
	prop.filter_spots = filter_spots;
	unsigned int flags = PAL::FILTER_SPOTS;
	PAL::SetCameraProperties(&prop, &flags);

	//27 = esc key. Run the loop until the ESC key is pressed
	while (key != 27)
	{
		PAL::Image left, right, depth, disparity;
		PAL::GrabFrames(&left, &right, &depth, &disparity, isDisparityNormalized, true);

		Mat temp, output;
		//Convert PAL::Image to Mat
		Mat l = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
		Mat r = Mat(right.rows, right.cols, CV_8UC3, right.Raw.u8_data);
		Mat d;

		if (isDisparityNormalized)
		{
			d = Mat(disparity.rows, disparity.cols, CV_8UC1, disparity.Raw.u8_data);
			cvtColor(d, d, cv::COLOR_GRAY2BGR);
		}
		else
		{
			d = Mat(disparity.rows, disparity.cols, CV_16SC1, disparity.Raw.u16_data);
			d.convertTo(d, CV_8UC1);
			cvtColor(d, d, cv::COLOR_GRAY2BGR);
		}

		//Vertical concatenation of left and right images into a temp
		vconcat(l, r, temp);

		//Vertical concatenation of temp and disparity into the final output
		vconcat(temp, d, output);

		//Display the final output image
		imshow("PAL window", output);

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;

		if (key == 'n' || key == 'N')
		{
			isDisparityNormalized = !isDisparityNormalized;
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

	}

	printf("exiting the application\n");
	PAL::Destroy();

	return 0;
}
