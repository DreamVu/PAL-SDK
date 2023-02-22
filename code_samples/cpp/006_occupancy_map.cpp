/*

CODE SAMPLE # 006: Occupancy Map
This code sample allows users to access the region map within a depth range.



>>>>>> Compile this code using the following command....

./compile.sh 006_occupancy_map.cpp

>>>>>> Execute the binary file by typing the following command...

./006_occupancy_map.out


>>>>>> KEYBOARD CONTROLS:
Press ESC to close the window
Press v/V to toggle vertical flip property    	
Press f/F to toggle filter rgb property

*/


#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "PAL.h"
#include <X11/Xlib.h>

using namespace cv;
using namespace std;


cv::Mat Getoccupancy1D(Mat rgb_image, Mat depth_mat, int depth_thresh, float context_threshold) 
{
	cv::Mat mask;
	
	cv::threshold(depth_mat, mask, depth_thresh, 255, cv::THRESH_BINARY_INV);
	mask = mask/255;
	cv::Mat occupancySum = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_32FC1);
	cv::reduce(mask, occupancySum, 0, cv::REDUCE_SUM, CV_32FC1);
	//initialize with zeros
	cv::Mat occupancy1D = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_8UC1); 
	cv::threshold(occupancySum, occupancy1D, rgb_image.rows*context_threshold/100.0, 255, cv::THRESH_BINARY_INV);
	occupancy1D = occupancy1D/255;
	occupancy1D.convertTo(occupancy1D, CV_8UC1);		

	return occupancy1D;
}

int main(int argc, char *argv[])
{
	// Create a window for display.
	namedWindow( "PAL Occupancy Map", WINDOW_NORMAL ); 


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

	//Connect to the PAL camera
	if (PAL::Init(width, height, camera_indexes, &def_mode) != PAL::SUCCESS) 
	{
		cout<<"Init failed"<<endl;
		return 1;
	}
	
	PAL::SetAPIMode(PAL::API_Mode::DEPTH);
	usleep(1000000);

	//discarding initial frames
	std::vector<PAL::Data::ODOA_Data> discard;
	for(int i=0; i<5;i++)
		discard =  PAL::GrabRangeScanData();		

	PAL::CameraProperties prop;
	PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &prop);

	if(ack_load != PAL::SUCCESS)
	{
		cout<<"Error Loading settings! Loading default values."<<endl;
	}
	
	//depth threshold in cm
	//The depth threshold should be kept within 1m to 2m range.
	int threshold_cm = (argc>2) ? atof(argv[2]) : 100;

	//context threshold in percentage to be considered for occupancy
	//The context threshold should be kept within 50(recommended) to 80 range.
	int context_threshold = (argc>3) ? atof(argv[3]) : 50;	

	if (threshold_cm > 200)
	{
		threshold_cm = 200;
		printf("depth threshold set above maximum range. Setting to 2m\n");
	}
	else if (threshold_cm < 100)
	{
		threshold_cm = 100;
		printf("depth threshold set below minimum range. Setting to 1m\n");
	}

	if (context_threshold > 80)
	{
		context_threshold = 80;
		printf("context threshold set above maximum range. Setting to 80\n");
	}
	else if(context_threshold < 50)
	{
		context_threshold = 50;
		printf("context threshold set below miminum range. Setting to 50\n");
	}

	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	// Getting Screen resolution 
	Display* disp = XOpenDisplay(NULL);
	Screen*  scrn = DefaultScreenOfDisplay(disp);
	int sc_height = scrn->height;
	int sc_width  = scrn->width;
	
	resizeWindow("PAL Occupancy Map", width, height);

	int key = ' ';

	printf("Press ESC to close the window\n");    
	printf("Press v/V to toggle vertical flip property\n");	
	printf("Press f/F to toggle filter rgb property\n");
	
	bool filter_spots = prop.filter_spots;
	bool flip = prop.vertical_flip;	
	
	//27 = esc key. Run the loop until the ESC key is pressed
	while(key != 27)
   	{
		std::vector<PAL::Data::ODOA_Data> data;

		data =  PAL::GrabRangeScanData();
		cv::Mat depth;
		if(prop.raw_depth)
			depth = data[0].fused_depth.clone();
		else
			depth = data[0].distance.clone();
				
		cv::Mat rgb = data[0].left.clone();
		

		depth.convertTo(depth, CV_8UC1);

		cv::Mat occupancy1D = Getoccupancy1D(rgb, depth,threshold_cm,context_threshold);

		cv::Mat r = cv::Mat::ones(1, rgb.cols, CV_8UC1);
		cv::Mat g = cv::Mat::ones(1, rgb.cols, CV_8UC1);
		cv::Mat b = cv::Mat::ones(1, rgb.cols, CV_8UC1);

		unsigned char* poccupancy1D = (unsigned char* )occupancy1D.data;
		unsigned char* pr = (unsigned char* )r.data;

		for(int i=0; i<rgb.cols; i++,poccupancy1D++,pr++)
		{
			if(!(*poccupancy1D))
			{
				*pr = (*pr)*2;
			}
		}
		
		cv::Mat final_img;
		vector<Mat> channels;
		channels.push_back(b);
		channels.push_back(g);
		channels.push_back(r);

		merge(channels, final_img);
		resize(final_img, final_img, rgb.size());
		cv::Mat colored_out = rgb.mul(final_img);
		
		imshow("PAL Occupancy Map", colored_out);
		
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
		if (key == 'v' || key == 'V')
		{		    
			PAL::CameraProperties prop;
			flip = !flip;
			prop.vertical_flip = flip;
			unsigned long int flags = PAL::VERTICAL_FLIP;
			PAL::SetCameraProperties(&prop, &flags);
		}
		
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}
