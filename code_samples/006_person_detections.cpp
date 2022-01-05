/*

CODE SAMPLE # 005: Switching Resolutions
This code sample allows users to access data proccessed at different resolutions.


>>>>>> Compile this code using the following command....


g++ 006_person_detections.cpp ../lib/libPAL.so ../lib/libPAL_CAMERA.so ../lib/libPAL_DEPTH.so ../lib/libPAL_SSD.so `pkg-config --libs --cflags opencv`   -g  -o 006_person_detections.out -I../include/ -lv4l2 -lpthread


>>>>>> Execute the binary file by typing the following command...


./005_resolutions.out

*/


# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"

using namespace cv;
using namespace std;

namespace PAL
{
	namespace Internal
	{
		void EnableDepth(bool flag);
	}
}

int main(int argc, char *argv[])
{

	int camera_index=1;
    cout << "Please enter you camera index(default is 1) and press Enter: ";
    cin >> camera_index;
	namedWindow( "PAL Person Detection", WINDOW_NORMAL ); // Create a window for display.

	//Enable/Disable depth alongside person detection	
	bool isDepthEnabled = false;
	PAL::Internal::EnableDepth(isDepthEnabled);
	
	int width, height;
	if(PAL::Init(width, height,camera_index) != PAL::SUCCESS) //Connect to the PAL camera
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

	//If a command line argument is passed then that will be taken as threshold
	float threshold = (argc>1) ? atof(argv[1]) : 0.5;

	if(PAL::InitPersonDetection(threshold)!= PAL::SUCCESS) //Initialise person detection pipeline
	{
		printf("Person Detection Init failed\n");
		return 1;
	}

	printf("The image resolution is .... %dx%d\n", width, height);


	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	resizeWindow("PAL Person Detection", width/4, (height/4)*1);

	int key = ' ';

	printf("Press ESC to close the window\n");    
	printf("Press r/R to toggle the restoration of rgb property\n");
	printf("Press f/F to toggle filter rgb property\n");
	
	size_t currentResolution = 0;
	std::vector<PAL::Resolution> resolutions = PAL::GetAvailableResolutions();
	bool filter_spots = false;	
	bool flip = false;

	//27 = esc key. Run the loop until the ESC key is pressed
	
	while(key != 27)
   	 {
		 
		cv::Mat rgb, depth, output;
		std::vector<PAL::BoundingBox> Boxes;  

		//Function to Query 
		//Image data: rgb & depth panoramas 
		//Person detection data: Bounding boxes of each person detected.    
		PAL::GetPeopleDetection(rgb, depth, &Boxes);

		int num_of_persons = Boxes.size();

		for(int i=0; i<num_of_persons; i++)
		{
			cv::rectangle(rgb,Point(Boxes[i].x1, Boxes[i].y1), Point(Boxes[i].x2, Boxes[i].y2), cv::Scalar(0,255,0), 5);
		}

#if isDepthEnabled
		depth.convertTo(depth, CV_8UC1);
		cvtColor(depth, depth, CV_GRAY2BGR);
		vconcat(rgb, depth, output);
		//Display the final rgb & depth image
		imshow( "PAL Person Detection", output); 
#else
		//Display the final rgb image
		imshow( "PAL Person Detection", rgb); 
#endif		

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;


		if (key == 'r' || key == 'R')
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
		if (key >= '1' && key <= ('0' + resolutions.size()))
		{
			int index = key - '1';
			PAL::CameraProperties prop;
			unsigned int flag = PAL::RESOLUTION;
			prop.resolution = resolutions[index];
			PAL::SetCameraProperties(&prop, &flag);
			printf("Changing the resolution to %dx%d\n",prop.resolution.width,prop.resolution.height);

		}
		
		if (key == 'f' || key == 'F')
		{	
			PAL::CameraProperties prop;
			prop.detection_mode = PAL::FLOOR;
			unsigned int flags = PAL::DETECTION_MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		
		
		if (key == 'i' || key == 'I')
		{	
			PAL::CameraProperties prop;
			prop.detection_mode = PAL::INTERMEDIATE;
			unsigned int flags = PAL::DETECTION_MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		
		if (key == 't' || key == 'T')
		{	
			PAL::CameraProperties prop;
			prop.detection_mode = PAL::TABLE_TOP;
			unsigned int flags = PAL::DETECTION_MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 'c' || key == 'C')
		{	
			PAL::CameraProperties prop;
			prop.detection_mode = PAL::CEILING;
			unsigned int flags = PAL::DETECTION_MODE;
			PAL::SetCameraProperties(&prop, &flags);
		}
		
		        
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}
