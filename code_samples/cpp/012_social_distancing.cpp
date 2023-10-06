/*

CODE SAMPLE # 012: Social Distancing
This code sample allows users to check if the distance between two people is within a limit or not.

>>>>>> Compile this code using the following command....

./compile.sh 012_social_distancing.cpp

>>>>>> Execute the binary file by typing the following command...

./012_social_distancing.out

*/

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "PAL.h"

using namespace cv;
using namespace std;

template <typename T> void GetUserInput(std::string text, T &value)
{
    while (true) 
    {
        std::cout << text << std::endl;
        if (std::cin >> value) 
        {
            if (std::cin.peek() == '\n')
            {
                break;
            }
        }

        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input. Please enter a valid value." << std::endl << std::endl;
    }
    return;
}

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
    //Distance threshold in cm
    //The Distance threshold should be kept within 1m to 2m range.
    float threshold_distance;
    std::string threshold_msg = "Enter the distance threshold in cm, eg 100";
    GetUserInput<float>(threshold_msg, threshold_distance);

    if(threshold_distance > 200)
    {
        threshold_distance = 200;
        cout<<"threshold distance set above maximum range. Setting to 2m"<<endl;
    }
    else if(threshold_distance < 100)
    {
        threshold_distance = 100;
        cout<<"threshold distance set below minumum range. Setting to 1m"<<endl;
    }

    //camera index is the video index assigned by the system to the camera. 
    //By default we set it to 5. Specify the index if the value has been changed.
    std::vector<int> camera_indexes{5};
    if(argc > 1) 
        camera_indexes[0] = std::atoi(argv[1]);
    
    //Connect to the PAL camera
    if (PAL::Init(camera_indexes) != PAL::SUCCESS) 
    {
        cerr<<"Init failed"<<endl;
        return 1;
    }

    //Setting API Mode
    PAL::SetAPIMode(PAL::API_Mode::TRACKING);
    
    //Loading camera properties from a text file
    PAL::CameraProperties properties;
    PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedProperties.yml", &properties);
    if(ack_load == PAL::Acknowledgement::INVALID_PROPERTY_VALUE)
    {
        PAL::Destroy();
        return 1;
    }
    if(ack_load != PAL::SUCCESS)
    {
        cerr<<"Error Loading settings! Loading default values."<<endl;
    }

    //Set depth detection mode
    {
        properties.depth_in_tracking = PAL::DepthInTracking::DEPTH_3DLOCATION_ON;
        unsigned long int flags = PAL::DEPTH_IN_TRACKING;
        PAL::SetCameraProperties(&properties, &flags);
    }

    //Set in which mode to run tracking
    int tracking_mode = PAL::Tracking_Mode::PEOPLE_DETECTION;
    int success = PAL::SetModeInTracking(tracking_mode);

    //Set minimum score threshold for detections. -1 as class id sets the same threshold for all classes
    float detection_threshold = 0.30;
    int class_id = -1;
    PAL::SetDetectionModeThreshold(detection_threshold, class_id);

    // Create a window for display.
    namedWindow( "PAL Social Distancing", WINDOW_AUTOSIZE);

    cout << "Press ESC to close the window." << endl;
    cout << "Press f/F to toggle filter rgb property" << endl;
    cout << "Press v/V to toggle Vertical Flip property." << endl;
    cout << "Press m/M to toggle Fast Depth property" << endl;
    cout << "Press q/Q & a/A to increase and decrease detection threshold respectively" << endl;

    std::vector<PAL::Data::Tracking_Data> data;

    int key = ' ';

    do
    {
        //Capturing Detection data from the camera
        data =  PAL::GrabTrackingData();

        cv::Mat display = data[0].left;
        
        int num_of_persons = data[0].tracking_info[PAL::States::OK].size();
        std::vector<bool> DistantData(num_of_persons, true);
        ComputeDistanceMatrix(data[0].tracking_info[PAL::States::OK], DistantData, threshold_distance);
        
        for(int i=0; i<num_of_persons; i++)
        {
            int x1,y1,x2,y2;
            x1 = (int)data[0].tracking_info[PAL::States::OK][i].boxes.x1;
            y1 = (int)data[0].tracking_info[PAL::States::OK][i].boxes.y1;
            x2 = (int)data[0].tracking_info[PAL::States::OK][i].boxes.x2;
            y2 = (int)data[0].tracking_info[PAL::States::OK][i].boxes.y2;

            if(DistantData[i])
                //Drawing GREEN box indicating the person is socially distant
                cv::rectangle(display, cv::Rect(x1, y1, x2, y2), cv::Scalar(0,255,0),2);
            else
                //Drawing RED box indicating the person is not socially distant 
                cv::rectangle(display, cv::Rect(x1, y1, x2, y2), cv::Scalar(0,0,255),2);
        }
        
        //Display the stereo images
        imshow( "PAL Social Distancing", display);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;

        if (key == 'q' || key == 'Q')
        {
            detection_threshold += 0.1;
            if(detection_threshold>1)
            {
                detection_threshold = 1;
            }
            std::cout << "Detection Threshold set to: " << detection_threshold << std::endl;

            PAL::SetDetectionModeThreshold(detection_threshold, class_id);
        }
        if (key == 'a' || key == 'A')
        {
            detection_threshold -= 0.1;
            if(detection_threshold<0.01)
            {
                detection_threshold = 0.01;
            }
            std::cout << "Detection Threshold set to: " << detection_threshold << std::endl;
            
            PAL::SetDetectionModeThreshold(detection_threshold, class_id);
        }
        
        if (key == 'f' || key == 'F')
        {
            properties.filter_spots = !properties.filter_spots;
            unsigned long int flags = PAL::FILTER_SPOTS;
            PAL::SetCameraProperties(&properties, &flags);
        }
        if (key == 'v' || key == 'V')
        {
            properties.vertical_flip = !properties.vertical_flip;
            unsigned long int flags = PAL::VERTICAL_FLIP;
            PAL::SetCameraProperties(&properties, &flags);
        }
        if (key == 'm' || key == 'M')
        {           
            properties.fd = !properties.fd;
            unsigned long int flags = PAL::FD;
            PAL::SetCameraProperties(&properties, &flags);
        }
    }
    //27 = esc key. Run the loop until the ESC key is pressed and camera is not changed
    while(key != 27 && !data[0].camera_changed);

    cout<<"exiting the application"<<endl;
    PAL::Destroy();
   
    return 0;
}
