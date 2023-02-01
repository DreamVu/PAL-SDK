/*

CODE SAMPLE # 004: PAL Video Capture
This code will grab the left & depth panorama and display in a window using opencv


>>>>>> Compile this code using the following command....

./compile.sh 004_video_capture.cpp

>>>>>> Execute the binary file by typing the following command...

./004_video_capture.out


>>>>>> KEYBOARD CONTROLS:

ESC key closes the window
Press v/V to toggle vertical flip property    
Press f/F to toggle filter rgb property       

*/


# include <stdio.h>
# include <opencv2/opencv.hpp>
# include "PAL.h"
#include "TimeLogger.h"
#include <time.h>
#include <unistd.h>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
	// Create a window for display.
	namedWindow( "PAL Video Capture", WINDOW_NORMAL ); 

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

	PAL::CameraProperties data;
	PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &data);

	if(ack_load != PAL::SUCCESS)
	{
		cout<<"Error Loading settings! Loading default values."<<endl;
	}

	//discarding initial frames
	std::vector<PAL::Data::ODOA_Data> discard;
	for(int i=0; i<5;i++)
		discard =  PAL::GrabRangeScanData();		

	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at otheir original resolution.
	resizeWindow("PAL Video Capture", width, 2*height);

	int key = ' ';
    bool record = false;
    bool closed = false;
    
	cout<<"Press ESC to close the window."<<endl;
	printf("Press v/V to toggle vertical flip property\n");	
	printf("Press f/F to toggle filter rgb property\n");
	
	bool filter_spots = data.filter_spots;
	bool flip = data.vertical_flip;	
	
	cout<<"Press C to capture a single frame into a PNG file."<<endl;
    cout<<"Press B to begin the video capture."<<endl;
    cout<<"Press E to end the video capture."<<endl;
    
    cv::VideoWriter video;
    
	Mat output = cv::Mat::zeros(2*height, width, CV_8UC3);

	//Display the overlayed image
	imshow( "PAL Video Capture", output);

	char image_filename[128];
	char video_filename[128];
	int image_count = 0;
	int video_count = 0;

	//27 = esc key. Run the loop until the ESC key is pressed
	while(!closed)
	{

		std::vector<PAL::Data::ODOA_Data> data;

		data =  PAL::GrabRangeScanData();	
		
		Mat l = data[0].left;
		Mat d = data[0].distance.clone();
		d.convertTo(d, CV_8UC1);
		cvtColor(d, d, cv::COLOR_GRAY2BGR);

		//Vertical concatenation of rgb and depth into the final output
		vconcat(l, d, output);

		//Display the depth with rgb panorama
		imshow( "PAL Video Capture", output);  

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
		if(key == 'C' || key == 'c') //capture an image using imwrite
        {
        	sprintf(image_filename, "image_%d.png", ++image_count);
            imwrite(image_filename, output);
            printf("The current frame is saved as image.png\n");
        }
        else if(key == 'B' || key == 'b')
        {
            cv::Size size = cv::Size(output.cols, output.rows);
            int fps = 15;
            printf("Opening the video\n");
            sprintf(video_filename, "pal_video_%d.avi", ++video_count);
            video = cv::VideoWriter(video_filename, cv::VideoWriter::fourcc('X','V','I','D'), fps, size);
            record = true;
        }
        else if (key == 'E' || key == 'e')
        {
            //closed = true;
            if(record)
            {
                record = false;            
                printf("Releasing the video \n");
                video.release();
            }
        }
        else if ( key == 27)
        {
            closed = true;
            if(record)
            {
                record = false;            
                printf("Releasing the video \n");
                video.release();
            }
        }
        if(record)
        { 
            video.write(output);
        }
        

	}

	printf("exiting the application\n");
	PAL::Destroy();

   
    return 0;
}

