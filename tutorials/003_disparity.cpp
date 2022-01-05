/*

PAL TUTORIAL # 003: Capturing the disparity panoramas in both normalized as well as un-normalized mode


This tutorial shows the minimal code required to...
1. Query the unnormalized 16 bit disparity.
2. Query the normalized 8 bit disparity.
3. Save the depth and disparity images

Compile this file using the following command....
g++ 003_disparity.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -lv4l2 -lpthread -g -o 003_disparity.out -I../include/

Run the output file using the following command....
./003_disparity.out

*/

# include <stdio.h>
# include <unistd.h>
# include <sys/types.h>



# include <opencv2/opencv.hpp>


# include "PAL.h"

using namespace std;
//Convert PAL::Image into cv::Mat
inline cv::Mat Convert(PAL::Image img)
{
	//CV_8UC3  is for 3 channel RGB image (left/right)
	int type = CV_8UC3;

	//In case if it is a single channelled image, 
	if (img.channels == 1)
	{
		switch (img.bytesPerChannel)
		{
		case 1:		type = CV_8UC1;			break; //8bit normalized disparity
		case 2:		type = CV_16SC1;		break; //16bit unnormalized disparity
		case 4:		type = CV_32FC1;		break; //32bit float depth
		}
	}

	return cv::Mat(img.rows, img.cols, type, img.Raw.u8_data);
}

int main(int argc, char** argv)
{
	//Same as previous tutorials
	int camera_index=1;
    cout << "Please enter you camera index(default is 1) and press Enter: ";
    cin >> camera_index;
	int image_width = -1;
	int image_height = -1;
    if(PAL::Init(image_width, image_height,camera_index) != PAL::SUCCESS)
        return 1; //Init failed        

	// *** NEW CODE *** specific to this tutorial
	//The disparity image would be single channel 8bit unsigned int data per pixel
	//Flag to indicate if the disparity image should be of 8bit normalized or 16 bit unnormalized
	bool normalize = true;

	// Flag to indicate if the function should...
	// return immediately (with the previous depth/disparity) or.. 
	// should get blocked till all the latest depth/disparity computation is finished
	bool asynchronous = false;

    PAL::Image left, right, depth, disparity;
	PAL::GrabFrames(&left, &right, &depth, &disparity, normalize, asynchronous);
	cv::Mat frame = Convert(disparity);
	cv::imwrite("disparity_normalized.png", frame);

	normalize = false;
	PAL::GrabFrames(&left, &right, &depth, &disparity, normalize, asynchronous);
	frame = Convert(disparity);
	cv::imwrite("disparity_unnormalized.png", frame);


	frame = Convert(depth);
	//Play with this number and see how the image changes
	float scale_factor = 1.0f / 1.6f; 
	frame = frame*scale_factor;
	cv::imwrite("depth.png", frame);

    PAL::Destroy();
	
	return 0;
}
