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




int main(int argc, char** argv)
{

	namedWindow("PAL window", WINDOW_NORMAL); // Create a window for display.

	int width, height;
	if (PAL::Init(width, height, -1) != PAL::SUCCESS) //Connect to the PAL camera
	{
		printf("Init failed\n");
		return 1;
	}


	PAL::CameraProperties prop;

	bool yuv = false;
	prop.color_space = yuv ? PAL::YUV444 : PAL::RGB;

	unsigned int flags = PAL::COLOR_SPACE;

	if (PAL::SetCameraProperties(&prop, &flags) != PAL::SUCCESS) //Connect to the PAL camera
	{
		printf("Setting colorspace failed\n");

	}

	bool isDisparityNormalized = true;
	if (argc == 1)
	{
		isDisparityNormalized = false;
		printf("Using the 16 bit disparity\n");
	}
	else
	{
		isDisparityNormalized = true;
		printf("Using the normalized 8 bit disparity\n");
	}

	printf("The image resolution is .... %dx%d\n", width, height);


	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	resizeWindow("PAL window", width / 4, (height / 4) * 3);

	int key = ' ';

	printf("Press ESC to close the window\n");
	printf("Press N to toggle the disparity normalization\n");
	printf("Press F to toggle the vertical flip of panorama\n");
	printf("Press Y to toggle the color space between RGB and YUV\n");


	//27 = esc key. Run the loop until the ESC key is pressed

	int counter = 0;
	size_t currentResolution = 0;
	PAL::CameraProperties data;

	bool flip = false;
        
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

		// The following is necessary because imshow expects a BGR color format.  	
		// See what happens if you disable this block.
		if (yuv)
		{
			cvtColor(l, l, cv::COLOR_YUV2BGR);
			cvtColor(r, r, cv::COLOR_YUV2BGR);
		}


		//Mat temp;
		//Vertical concatenation of left and right images into a temp
		vconcat(l, r, temp);

		//Vertical concatenation of temp and disparity into the final output
		vconcat(temp, d, output);

		//Display the final output image
		imshow("PAL window", output);

		//imshow( "PAL window", d);  

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;

		if (key == 'n' || key == 'N')
		{
			isDisparityNormalized = !isDisparityNormalized;
		}

		if (key == 'f' || key == 'F')
		{
			flip = !flip;
			prop.vertical_flip = flip;
			unsigned int flags = PAL::VERTICAL_FLIP;
			PAL::SetCameraProperties(&prop, &flags);
		}

		if (key == 'y' || key == 'Y')
		{
			yuv = !yuv;
			prop.color_space = yuv ? PAL::YUV444 : PAL::RGB;

			unsigned int flags = PAL::COLOR_SPACE;
			PAL::SetCameraProperties(&prop, &flags);
			if (yuv)
				printf("Now using YUV color space.\n");
			else
				printf("Now using RGB color space.\n");
		}


	}

	printf("exiting the application\n");
	PAL::Destroy();

	return 0;
}
