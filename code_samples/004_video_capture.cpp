/*

CODE SAMPLE # 004: PAL Video Capture
This code will grab the left & depth panorama and display in a window using opencv


>>>>>> Compile this code using the following command....

./compile.sh

>>>>>> Execute the binary file by typing the following command...

./004_video_capture.out


>>>>>> KEYBOARD CONTROLS:

       ESC key closes the window
       

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

	namedWindow( "PAL Video Capture", WINDOW_NORMAL ); // Create a window for display.

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

	PAL::SetAPIMode(PAL::API_Mode::DEPTH);
	usleep(1000000);

	PAL::CameraProperties data;
	PAL::Acknowledgement ack_load = PAL::LoadProperties("../Explorer/SavedPalProperties.txt", &data);

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

	cout<<"Press C to capture a single frame into a PNG file."<<endl;
    cout<<"Press B to begin the video capture."<<endl;
    cout<<"Press E to end the video capture."<<endl;
    
    cv::VideoWriter video;
    
	Mat output = cv::Mat::zeros(2*height, width, CV_8UC3);

	//Display the overlayed image
	imshow( "PAL Video Capture", output);

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
		
		if(key == 'C' || key == 'c') //capture an image using imwrite
        {
            imwrite("image.png", output);
            printf("The current frame is saved as image.png\n");
        }
        else if(key == 'B' || key == 'b')
        {
            cv::Size size = cv::Size(output.cols, output.rows);
            int fps = 15;
            printf("Opening the video\n");
            video = cv::VideoWriter("pal_video.avi",cv::VideoWriter::fourcc('X','V','I','D'), fps, size);
            record = true;
        }
        else if (key == 'E' || key == 'e' || key == 27)
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

