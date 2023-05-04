/*

CODE SAMPLE # 017: 3D Location
This code will grab the left panorama with person detection data and overlay their 3D location


>>>>>> Compile this code using the following command....

./compile.sh 017_3d_person_locations.cpp


>>>>>> Execute the binary file by typing the following command...

./017_3d_person_locations.out


>>>>>> KEYBOARD CONTROLS:

    Press ESC to close the window.
    Press f/F to toggle filter rgb property
    Press v/V to toggle Vertical Flip property.
    Press m/M to toggle Fast Depth property        
*/


# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"
#include "TimeLogger.h"
#include <time.h>
#include <unistd.h>

#include<sys/time.h>

#include <iomanip>

using namespace cv;
using namespace std;

void setLabel(cv::Mat& input, const std::string label, const cv::Point org, cv::Scalar clr)
{
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 2;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::rectangle(input, org + cv::Point(0, baseline), org + cv::Point(text.width, -text.height), CV_RGB(0,0,0), cv::FILLED);
    cv::putText(input, label, org, fontface, scale, clr, thickness, 4);
}

void draw3DLocation(cv::Mat &rgb, const PAL::Data::TrackingResults &data)
{
    int no_of_persons = data.trackingData[PAL::States::OK].size();
    for(int i =0; i<no_of_persons; i++)
    {
        int x1,y1,x2,y2;
        x1 = (int)data.trackingData[PAL::States::OK][i].boxes.x1;
        y1 = (int)data.trackingData[PAL::States::OK][i].boxes.y1;
        x2 = (int)data.trackingData[PAL::States::OK][i].boxes.x2;
        y2 = (int)data.trackingData[PAL::States::OK][i].boxes.y2;

        float x3D, y3D, z3D, depth_value;
        x3D = data.trackingData[PAL::States::OK][i].locations_3d.x;
        y3D = data.trackingData[PAL::States::OK][i].locations_3d.y;
        z3D = data.trackingData[PAL::States::OK][i].locations_3d.z;

        depth_value = sqrt(x3D*x3D + y3D*y3D);

        char text[128];
        cv::Scalar color;
        sprintf(text, "x:%.1fm y:%.1fm z:%.1fm", x3D, y3D, z3D);

        if(depth_value > 3)
        {
            color = cv::Scalar(0,255,0);
        }
        else
        {
            color = cv::Scalar(0,0,255);
        }
        
        cv::circle(rgb, cv::Point(x1+x2/2, y1+y2/4), 5, color, -1);
        setLabel(rgb, text, cv::Point(x1+x2/2, y1+y2/4-10), color);
    }
}

int main( int argc, char** argv )
{
    namedWindow( "PAL 3D Location", WINDOW_NORMAL ); // Create a window for display.

    int width, height;
    std::vector<int> camera_indexes{5};
    PAL::Mode def_mode = PAL::Mode::LASER_SCAN;

    //Start the PAL application
    if (PAL::Init(width, height, camera_indexes, &def_mode) != PAL::SUCCESS) //Connect to the PAL camera
    {
        cout<<"Init failed"<<endl;
        return 1;
    }

    //Select which mode you want to run the application in.
    PAL::SetAPIMode(PAL::API_Mode::TRACKING);
    usleep(1000000);

    PAL::CameraProperties cam_data;
    PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &cam_data);

    if(ack_load != PAL::SUCCESS)
    {
        cout<<"Error Loading settings! Loading default values."<<endl;
    }

    bool filter_spots = cam_data.filter_spots;
    bool flip = cam_data.vertical_flip;
    bool fd = cam_data.fd;

    PAL::SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_3DLOCATION_ON);

    int tracking_mode = PAL::Tracking_Mode::PEOPLE_DETECTION;
    int success = PAL::SetModeInTracking(tracking_mode);

    float detection_threshold = 0.30;
    int class_id = -1; // -1 means all classes
    PAL::SetDetectionModeThreshold(detection_threshold, class_id);

	std::vector<PAL::Data::TrackingResults> dataDiscard;
	dataDiscard =  PAL::GrabTrackingData();    

	width = dataDiscard[0].left.cols;
	height = dataDiscard[0].left.rows;


    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed at their original resolution.
    resizeWindow("PAL 3D Location", width, height);

    int key = ' ';

    cout << "Press ESC to close the window." << endl;
    cout << "Press f/F to toggle filter rgb property" << endl;
    cout << "Press v/V to toggle Vertical Flip property." << endl;
    cout << "Press m/M to toggle Fast Depth property" << endl;
    cout << "Press q/Q & a/A to increase and decrease detection threshold respectively\n\n" << endl;
	
	//27 = esc key. Run the loop until the ESC key is pressed
	while(key != 27)
	{


        std::vector<PAL::Data::TrackingResults> data;
        data =  PAL::GrabTrackingData();    
		if(data[0].camera_changed)
		{
			break;
		}
        cv::Mat display = data[0].left;
        draw3DLocation(display, data[0]);
        
        //Display the stereo images
        imshow( "PAL 3D Location", display);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;

        if(key == 'q' || key == 'Q')
        {
            detection_threshold += 0.1;
            if(detection_threshold>1)
            {
                detection_threshold = 1;
                std::cout << "Max threshold (1.0) reached" << std::endl;
            }
            PAL::SetDetectionModeThreshold(detection_threshold, class_id);
        }

        if(key == 'a' || key == 'A')
        {
            detection_threshold -= 0.1;
            if(detection_threshold<0.01)
            {
                detection_threshold = 0.01;
                std::cout << "Min threshold (0.01) reached" << std::endl;
            }
            PAL::SetDetectionModeThreshold(detection_threshold, class_id);
        }
        
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

        if (key == 'm' || key == 'M')
        {           
            PAL::CameraProperties prop;
            fd = !fd;
            prop.fd = fd;
            unsigned long int flags = PAL::FD;
            PAL::SetCameraProperties(&prop, &flags);
        }
    }

    printf("exiting the application\n");
    PAL::Destroy();
   
    return 0;
}

