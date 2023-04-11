/*

CODE SAMPLE # 012: Social Distancing
This code sample allows users to check if the distance between two people is within a limit or not.


>>>>>> Compile this code using the following command....

./compile.sh 012_social_distancing.cpp


>>>>>> Execute the binary file by typing the following command...

./012_social_distancing.out


>>>>>> KEYBOARD CONTROLS:

    Press ESC to close the window.
    Press f/F to toggle filter rgb property
    Press v/V to toggle Vertical Flip property.
    Press m/M to toggle Fast Depth property.
    Press up/down arrow key to increase/decrease detection threshold respectively
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

//Function to compute distance between two persons
bool IsSociallyDistant(PAL::Loc3D p1, PAL::Loc3D p2, float threshold)
{

    if((sqrt(pow(p1.x-p2.x,2.0)+pow(p1.y-p2.y,2.0)+pow(p1.z-p2.z,2.0))) <= threshold)
        return false;
    return true;     
}

//Function to compute whether the detected persons are socially distant or not
void ComputeDistanceMatrix(std::vector<PAL::Data::TrackND> track_info, std::vector<bool>& DistantData, float threshold_distance)
{     
    int num_persons = track_info.size();
    bool b = true;

    for(int i=0; i<num_persons; i++)
    {
        for(int j = i+1; j<num_persons; j++)
        {     
            //checking if location of two persons are larger or not than given threshold
            //threshold distance is in cm and the distance returned by the api is in m. Hence the division by 100.
            b = IsSociallyDistant(track_info[i].locations_3d, track_info[j].locations_3d, threshold_distance/100);
            if(!b)
                DistantData[i] = DistantData[j] = false;
        }
    }
}


int main( int argc, char** argv )
{
    namedWindow( "PAL Social Distance", WINDOW_NORMAL ); // Create a window for display.

    //Select the Model to use in Tracking. To be set before Init call.
    PAL::SetInitTrackingModel(PAL::Tracking_Model::MODEL_0);

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

    bool enableDepth = true;
    PAL::SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_3DLOCATION_ON);

    float threshold_distance = (argc>1) ? atof(argv[1]) : 100.0f; //threshold_distane should be between 1m to 2m.

    if(threshold_distance > 200)
    {
        threshold_distance = 200;
        printf("threshold distance set above maximum range. Setting to 2m\n");
    }
    else if(threshold_distance < 100)
    {
        threshold_distance = 100;
        printf("threshold distance set below minumum range. Setting to 1m\n");
    }

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
    //Each of the panoramas are displayed at otheir original resolution.
    resizeWindow("PAL Social Distance", width, height);

    int key = ' ';

    cout << "Press ESC to close the window." << endl;
    cout << "Press f/F to toggle filter rgb property" << endl;
    cout << "Press v/V to toggle Vertical Flip property." << endl;
    cout << "Press m/M to toggle Fast Depth property" << endl;
    cout << "Press up/down arrow key to increase/decrease detection threshold respectively" << endl;
	extern bool camera_changed;
	
	//27 = esc key. Run the loop until the ESC key is pressed
	while(key != 27)
	{
		
		if(camera_changed)
		{
			break;
		}

        std::vector<PAL::Data::TrackingResults> data;
        data =  PAL::GrabTrackingData();

        cv::Mat display = data[0].left;
        int num_of_persons = data[0].trackingData[PAL::States::OK].size();
        std::vector<bool> DistantData(num_of_persons, true);

        if(num_of_persons>=2)
        {
            ComputeDistanceMatrix(data[0].trackingData[PAL::States::OK], DistantData, threshold_distance);
            for(int i=0; i<num_of_persons; i++)
            {
                int x1,y1,x2,y2;
                x1 = (int)data[0].trackingData[PAL::States::OK][i].boxes.x1;
                y1 = (int)data[0].trackingData[PAL::States::OK][i].boxes.y1;
                x2 = (int)data[0].trackingData[PAL::States::OK][i].boxes.x2;
                y2 = (int)data[0].trackingData[PAL::States::OK][i].boxes.y2;

                if(DistantData[i])
                    //Drawing GREEN box indicating the person is socially distant
                    cv::rectangle(display, cv::Rect(x1, y1, x2, y2), cv::Scalar(0,255,0),2);
                else
                    //Drawing RED box indicating the person is not socially distant 
                    cv::rectangle(display, cv::Rect(x1, y1, x2, y2), cv::Scalar(0,0,255),2);
            }  
        }
        else if(num_of_persons==1)
        {
            int x1,y1,x2,y2;
            x1 = (int)data[0].trackingData[PAL::States::OK][0].boxes.x1;
            y1 = (int)data[0].trackingData[PAL::States::OK][0].boxes.y1;
            x2 = (int)data[0].trackingData[PAL::States::OK][0].boxes.x2;
            y2 = (int)data[0].trackingData[PAL::States::OK][0].boxes.y2;
            cv::rectangle(display, cv::Rect(x1, y1, x2, y2), cv::Scalar(0,255,0), 2);
        }
        
        //Display the stereo images
        imshow( "PAL Social Distance", display);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;

        if(key == 82) //up arrow key
        {
            detection_threshold += 0.1;
            if(detection_threshold>1)
            {
                detection_threshold = 1;
                std::cout << "Max threshold (1.0) reached" << std::endl;
            }
            PAL::SetDetectionModeThreshold(detection_threshold, class_id);
        }

        if(key == 84) //down arrow key
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

