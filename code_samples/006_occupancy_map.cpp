/*

CODE SAMPLE # 006: Occupancy Map
This code sample allows users to access the region map within a depth range.



>>>>>> Compile this code using the following command....

g++  006_occupancy_map.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DE.so ../lib/libPAL_EDET.so `pkg-config --libs --cflags opencv`   -O3  -o  006_occupancy_map.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl -lstdc++fs -lX11

>>>>>> Execute the binary file by typing the following command...

./006_occupancy_map.out


>>>>>> KEYBOARD CONTROLS:
	Press ESC to close the window
	Press v/V to toggle vertical flip property
	Press f/F to toggle filter rgb property
	Press d/D to toggle fast depth property
	Press r/R to toggle near range property

*/


# include <stdio.h>
# include <opencv2/opencv.hpp>
# include "PAL.h"
#include <X11/Xlib.h>
using namespace cv;
using namespace std;

namespace PAL
{
	namespace Internal
	{
		void EnableDepth(bool flag);
		void MinimiseCompute(bool flag);
	}
}

cv::Mat Getoccupancy1D(Mat rgb_image, Mat depth_mat, int depth_thresh, float context_threshold) 
{
	cv::Mat mask;
	
	cv::threshold(depth_mat, mask, depth_thresh, 255, cv::THRESH_BINARY_INV);
	mask = mask/255;
	cv::Mat occupancySum = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_32FC1);
	cv::reduce(mask, occupancySum, 0, cv::REDUCE_SUM, CV_32FC1);
	cv::Mat occupancy1D = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_8UC1); //initialize with zeros;
	cv::threshold(occupancySum, occupancy1D, rgb_image.rows*context_threshold/100.0, 255, cv::THRESH_BINARY_INV);
	occupancy1D = occupancy1D/255;
	occupancy1D.convertTo(occupancy1D, CV_8UC1);		

	return occupancy1D;
}

int main(int argc, char *argv[])
{
	namedWindow( "PAL Occupancy Map", WINDOW_NORMAL ); // Create a window for display.

	//Depth should be enable for occupancy map as a prerequisite	
	bool isDepthEnabled = true;
	PAL::Internal::EnableDepth(isDepthEnabled);
	PAL::Internal::MinimiseCompute(false);
	int width, height;
	if(PAL::Init(width, height,-1) != PAL::SUCCESS) //Connect to the PAL camera
	{
		printf("Camera Init failed\n");
		return 1;
	}

	PAL::CameraProperties data; 
	PAL::Acknowledgement ack = PAL::LoadProperties("../Explorer/SavedPalProperties.txt", &data);
	if(ack != PAL::SUCCESS)
	{
	    printf("Error Loading settings\n");
	}

	PAL::CameraProperties prop;
	
	unsigned int flag = PAL::MODE | PAL::FD | PAL::NR | PAL::FILTER_SPOTS | PAL::VERTICAL_FLIP;
	prop.mode = PAL::Mode::POINT_CLOUD_25D; // The other available option is PAL::Mode::HIGH_QUALITY_DEPTH
	prop.fd = 1;
	prop.nr = 1;
	prop.filter_spots = 1;
	prop.vertical_flip =0;
	PAL::SetCameraProperties(&prop, &flag);

	//depth threshold in cm
	//The depth threshold should be kept within 1m to 2m range.
	int threshold_cm = (argc>1) ? atof(argv[1]) : 100;

	//context threshold in percentage to be considered for occupancy
	//The context threshold should be kept within 50(recommended) to 80 range.
	int context_threshold = (argc>2) ? atof(argv[2]) : 50;	

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
	
	resizeWindow("PAL Occupancy Map", sc_width, sc_height);//width/4, (height/4)*2);

	int key = ' ';

	printf("Press ESC to close the window\n");    
	printf("Press v/V to toggle vertical flip property\n");
	printf("Press f/F to toggle filter rgb property\n");
	printf("Press r/R to toggle near range property\n");
	
	size_t currentResolution = 0;
	//std::vector<PAL::Resolution> resolutions = PAL::GetAvailableResolutions();
	bool filter_spots = true;	
	bool flip = false;
	bool nr = true;
	bool fd = true;

	//27 = esc key. Run the loop until the ESC key is pressed
	
	while(key != 27)
   	{
		cv::Mat rgb, depth, output, right;
		PAL::Image left_img, right_img, depth_img, disparity;
		
		PAL::GrabFrames(&left_img, &right_img, &depth_img);
		
		depth = cv::Mat(depth_img.rows, depth_img.cols, CV_32FC1, depth_img.Raw.f32_data);
		rgb = cv::Mat(left_img.rows, left_img.cols, CV_8UC3, left_img.Raw.u8_data);
		cv::resize(depth, depth , cv::Size(rgb.cols, rgb.rows));
		
		PAL::CameraProperties data;
		PAL::GetCameraProperties(&data);
		
		
        if(isDepthEnabled)      
        {

			depth.convertTo(depth, CV_8UC1);
			
			cv::Mat occupancy1D = Getoccupancy1D(rgb, depth,threshold_cm,context_threshold);

			cv::Mat r = cv::Mat::ones(1, rgb.cols, CV_8UC1);
 			cv::Mat g = cv::Mat::ones(1, rgb.cols, CV_8UC1);
			cv::Mat b = cv::Mat::ones(1, rgb.cols, CV_8UC1);

  			unsigned char* poccupancy1D = (unsigned char* )occupancy1D.data;
  			unsigned char* pr = (unsigned char* )r.data;

			for(int i=0; i<rgb.cols; i++,poccupancy1D++,pr++)
			{
				if((*poccupancy1D))
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
			imshow("PAL Occupancy Map", colored_out);//colored_out);
        }
        else
        {	
            //Display the final rgb image
            imshow( "PAL Occupancy Map", rgb); 
        }		

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
