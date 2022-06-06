/*

PAL TUTORIAL # 001: Writing PNG images using OpenCV

This tutorial shows the minimal code required to...
1. Grab stereo panoramic images from PAL API
2. Convert PAL images into OpenCV Mat
3. Write the OpenCV Mat into png files.


Compile this file using the following command....
g++ 001_cv_png.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DE.so ../lib/libPAL_EDET.so   `pkg-config --libs --cflags opencv`   -lv4l2 -lpthread -O3 -o 001_cv_png.out -I../include/

Run the output file using the following command....
./001_cv_png.out

*/

# include <stdio.h>
# include <unistd.h>
# include <sys/types.h>
# include <opencv2/opencv.hpp>

# include "PAL.h"


//Convert PAL::Image into cv::Mat
inline cv::Mat Convert(PAL::Image img)
{
	//CV_8UC3  is for 3 channel RGB image (for e.g., left/right panorama)
	int type = CV_8UC3;


	return cv::Mat(img.rows, img.cols, type, img.Raw.u8_data);
}


int main(int argc, char** argv)
{

	//////PAL related code

	//This should be same as the camera index for OpenCV video capture.
	//When -1 is used, PAL API would try to automatically detect the index of PAL camera.
	int camera_index = -1;

	//These variables would be assigned/updated inside PAL::Init function (call by reference)
	int image_width = -1;
	int image_height = -1;

	if (PAL::Init(image_width, image_height, camera_index) != PAL::SUCCESS)
	{
		printf("Unable to initialize PAL camera. Please make sure that the camera is connected\n");
		return 1; //Init failed
	}

	PAL::Image left, right;
	cv::Mat frame;

	PAL::GrabFrames(&left, &right);

	frame = Convert(left);
	cv::imwrite("left.png", frame);

	frame = Convert(right);
	cv::imwrite("right.png", frame);

	printf("Destroying...\n");
	PAL::Destroy();

	return 0;
}
