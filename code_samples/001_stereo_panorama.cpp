/*

   CODE SAMPLE # 001: Basic stereo panorama
   This code will grab the basic stereo panoramas (left and right images) and would be displayed in a window using opencv


   >>>>>> Compile this code using the following command....


   g++ 001_stereo_panorama.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -g  -o 001_stereo_panorama.out -I../include/ -lv4l2 -lpthread


   >>>>>> Execute the binary file by typing the following command...


   ./001_stereo_panorama.out


   >>>>>> KEYBOARD CONTROLS:

   ESC key closes the window
   Press f/F to toggle filter rgb property
	Press v/V to toggle vertical flip property


*/


# include <stdio.h>
# include <opencv2/opencv.hpp>
# include "PAL.h"

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
	namedWindow( "PAL Stereo Panorama", WINDOW_NORMAL ); // Create a window for display.

	int width, height;
	if(PAL::Init(width, height,-1) != PAL::SUCCESS) //Connect to the PAL camera
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
	flag = flag | PAL::FILTER_SPOTS;
	flag = flag | PAL::VERTICAL_FLIP;
	    
	prop.mode = PAL::Mode::STEREO;
	prop.filter_spots = 1;
	prop.vertical_flip =0;
	PAL::SetCameraProperties(&prop, &flag);
	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at one fourth their original resolution.
	//Since the panoramas are vertically stacked, the window height should be twice of 1/4th height
	resizeWindow("PAL Stereo Panorama", width/4, (height/4)*2);

	printf("Press ESC to close the window.\n");
	printf("Press f/F to toggle filter rgb property\n");
	printf("Press v/V to toggle vertical flip property\n");

	int key = ' ';

	bool filter_spots = true;   
	bool flip = false;

	Mat output = cv::Mat::zeros(height, width, CV_8UC3);

	//Display the concatenated image
	imshow( "PAL Stereo Panorama", output);

	//27 = esc key. Run the loop until the ESC key is pressed
	while(key != 27)
	{
		PAL::Image left, right;
		PAL::GrabFrames(&left, &right, 0);

		//Convert PAL::Image to Mat
		Mat l = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
		Mat r = Mat(right.rows, right.cols, CV_8UC3, right.Raw.u8_data);

		//Vertical concatenation of left and right images
		vconcat(l, r, output);

		//Display the concatenated image
		imshow( "PAL Stereo Panorama", output);  

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
	}

	printf("exiting the application\n");
	PAL::Destroy();

	return 0;
}
