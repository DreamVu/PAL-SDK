/*

   CODE SAMPLE # 001: Basic stereo panorama
   This code will grab the basic stereo panoramas (left and right images) and would be displayed in a window using opencv


   >>>>>> Compile this code using the following command....


  g++ 001_stereo_panorama.cpp  /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DE.so ../lib/libPAL_EDET.so  `pkg-config --libs --cflags opencv`    -O3  -o 001_stereo_panorama.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl -lX11


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
#include <X11/Xlib.h>
using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
	namedWindow( "PAL Stereo Panorama", WINDOW_NORMAL ); // Create a window for display.


	cout<<"Testing for case no. 9 and 10\n";
	cout<<"Second Testing for case no. 9 and 10\n";
	

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
	
	// Getting Screen resolution 
	Display* disp = XOpenDisplay(NULL);
	Screen*  scrn = DefaultScreenOfDisplay(disp);
	int sc_height = scrn->height;
	int sc_width  = scrn->width;
	
	resizeWindow("PAL Stereo Panorama", sc_width-60, sc_height-60);//width/4, (height/4)*2);

	printf("Press ESC to close the window.\n");
	printf("Press f/F to toggle filter rgb property\n");
	printf("Press v/V to toggle vertical flip property\n");

	int key = ' ';

	bool filter_spots = true;   
	bool flip = false;

	Mat output = cv::Mat::zeros(height, width, CV_8UC3);

	//Display the concatenated image
	//imshow( "PAL Stereo Panorama", output);

	//27 = esc key. Run the loop until the ESC key is pressed
	while(key != 27)
	{
		
		timeval timestamp;
		cv::Mat output = PAL::GetCroppedStereo(5290, 3638, 0, 0, timestamp,1);
		
		//Display the concatenated image
		//imshow( "PAL Stereo Panorama", output);  
		imshow("PAL Stereo Panorama",output);

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
