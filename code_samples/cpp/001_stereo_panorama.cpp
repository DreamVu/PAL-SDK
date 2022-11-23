/*

CODE SAMPLE # 001: Stereo Panorama
This code will grab the left & right panorama and display in a window using opencv


>>>>>> Compile this code using the following command....

./compile.sh 001_stereo_panorama.cpp


>>>>>> Execute the binary file by typing the following command...

./001_stereo_panorama.out


>>>>>> KEYBOARD CONTROLS:

    ESC key closes the window
	Press f/F to toggle filter rgb property       
       

*/


# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"
#include "TimeLogger.h"
#include <time.h>
#include <unistd.h>
#include <X11/Xlib.h>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{

	namedWindow( "PAL Stereo Panorama", WINDOW_NORMAL ); // Create a window for display.

	int width, height;
	PAL::Mode mode = PAL::Mode::LASER_SCAN;

	std::vector<int> camera_indexes{5};
	
	if(argc > 1) 
		camera_indexes[0] = std::atoi(argv[1]);


	PAL::Mode def_mode = PAL::Mode::LASER_SCAN;

	char path[1024];
	sprintf(path,"/usr/local/bin/data/pal/data%d/",camera_indexes[0]);

	char path2[1024];
	sprintf(path2,"/usr/local/bin/data/pal/data%d/",6);

	PAL::SetPathtoData(path, path2);

	if (PAL::Init(width, height, camera_indexes, &def_mode) != PAL::SUCCESS) //Connect to the PAL camera
	{
		cout<<"Init failed"<<endl;
		return 1;
	}
	
	PAL::SetAPIMode(PAL::API_Mode::STEREO);
	
	usleep(1000000);

	PAL::CameraProperties data;
	PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &data);

	if(ack_load != PAL::SUCCESS)
	{
		cout<<"Error Loading settings! Loading default values."<<endl;
	}

	//discarding initial frames
	//std::vector<PAL::Data::ODOA_Data> discard;
	//for(int i=0; i<5;i++)
		//discard =  PAL::GrabRangeScanData();		

	// Getting Screen resolution 
	Display* disp = XOpenDisplay(NULL);
	Screen*  scrn = DefaultScreenOfDisplay(disp);
	int sc_height = scrn->height;
	int sc_width  = scrn->width;
	
	resizeWindow("PAL Stereo Panorama", sc_width-60, sc_height-60);
	
	int key = ' ';

	cout<<"Press ESC to close the window."<<endl;
	printf("Press f/F to toggle filter rgb property\n");

	Mat output = cv::Mat::zeros(height, width, CV_8UC3);
	bool filter_spots = true;	
	//Display the overlayed image
	imshow( "PAL Stereo Panorama", output);

	//27 = esc key. Run the loop until the ESC key is pressed
	while(key != 27)
	{

		std::vector<PAL::Data::Stereo> data;

		data =  PAL::GetStereoData();	

        cv::Mat display;
        
        vconcat(data[0].stereo_left, data[0].stereo_right, display);
        
		//Display the stereo images
		imshow( "PAL Stereo Panorama", display);  

		//Wait for the keypress - with a timeout of 1 ms

		key = waitKey(1) & 255;
		
		if (key == 'f' || key == 'F')
		{	
			PAL::CameraProperties prop;
			filter_spots = !filter_spots;
			prop.filter_spots = filter_spots;
			unsigned long int flags = PAL::FILTER_SPOTS;
			PAL::SetCameraProperties(&prop, &flags);
		}

	}

	printf("exiting the application\n");
	PAL::Destroy();

   
    return 0;
}

