/*

PAL TUTORIAL # 002: Changinging the camera properties

This tutorial shows the minimal code required to...
1. Query PAL API for the current camera properties.
2. Change the camera properties
3. Write the panoramas captured with different camera properties


Compile this file using the following command....
g++ 002_set_properties.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DE.so ../lib/libPAL_EDET.so  `pkg-config --libs --cflags opencv` -lv4l2 -lpthread  -O3 -o 002_set_properties.out -I../include/


Run the output file using the following command....
./002_set_properties.out


*/
# include <stdio.h>
# include <unistd.h>
# include <sys/types.h>


# include <opencv2/opencv.hpp>


# include "PAL.h"


//Same as previous tutorial
inline cv::Mat Convert(PAL::Image img)
{
	//CV_8UC3  is for 3 channel RGB image (for e.g., left/right panorama)
	int type = CV_8UC3;

	return cv::Mat(img.rows, img.cols, type, img.Raw.u8_data);
}


int main(int argc, char** argv)
{

	//Same as previous tutorial
	int camera_index = -1;
    int image_width = -1;
    int image_height = -1;
    if(PAL::Init(image_width,image_height, camera_index) == PAL::FAILURE)
        return 1; //Init failed


	// ***** NEW CODE - SPECIFIC TO THIS TUTORIAL *****
	//Get the current camera properties
	PAL::CameraProperties properties;
    PAL::GetCameraProperties(&properties);

	//Modify the camera properties
	properties.auto_white_bal = false;
    properties.white_bal_temp = PAL::CameraProperties::MIN_WHITE_BAL_TEMP;
    PAL::SetCameraProperties(&properties);


	//Same as previous tutorial
	PAL::Image left, right;
	//Let the camera take some time to change the properties
	for(int i = 0; i < 10; i++)
		PAL::GrabFrames(&left, &right);
	cv::Mat frame = Convert(left);
	cv::imwrite("left1.png", frame);


	// ***** NEW CODE - SPECIFIC TO THIS TUTORIAL *****
	properties.white_bal_temp = PAL::CameraProperties::MAX_WHITE_BAL_TEMP;
	PAL::SetCameraProperties(&properties);


	//Same as previous tutorial
	for(int i = 0; i < 10; i++)
		PAL::GrabFrames(&left, &right);
	frame = Convert(left);
	cv::imwrite("left2.png", frame);
    PAL::Destroy();
    
    return 0;
}
