/*

CODE SAMPLE # 013: Safe Zone Detection
This code sample allows users to run Safe Zone Detection.

>>>>>> Compile this code using the following command....

./compile.sh 013_safe_zone_detection.cpp

>>>>>> Execute the binary file by typing the following command...

./013_safe_zone_detection.out

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

void setLabel(cv::Mat& input, const std::string label, const cv::Point org, cv::Scalar clr)
{
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.8;
    int thickness = 2;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::rectangle(input, org + cv::Point(0, baseline), org + cv::Point(text.width, -text.height), CV_RGB(0,0,0), cv::FILLED);
    cv::putText(input, label, org, fontface, scale, clr, thickness, 4);
}

int main( int argc, char** argv )
{
    //Distance threshold in cm
    //The Distance threshold should be kept within 1m to 2m range.
    float safe_distance;
    std::string safe_distance_msg = "Enter the value marking the safe distance in cm, eg 100";
    GetUserInput<float>(safe_distance_msg, safe_distance);
    
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
    PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &properties);
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
    PAL::SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_3DLOCATION_ON);

    //Set in which mode to run tracking
    int tracking_mode = PAL::Tracking_Mode::PEOPLE_DETECTION;
    int success = PAL::SetModeInTracking(tracking_mode);

    //Set minimum score threshold for detections. -1 as class id sets the same threshold for all classes
    float detection_threshold = 0.30;
    int class_id = -1;
    PAL::SetDetectionModeThreshold(detection_threshold, class_id);

    // Create a window for display.
    namedWindow( "PAL Safe Zone Distance", WINDOW_AUTOSIZE);

    cout << "Press ESC to close the window." << endl;
    cout << "Press f/F to toggle filter rgb property" << endl;
    cout << "Press v/V to toggle Vertical Flip property." << endl;
    cout << "Press m/M to toggle Fast Depth property" << endl;
    cout << "Press q/Q & a/A to increase and decrease detection threshold respectively" << endl;

    std::vector<PAL::Data::TrackingResults> data;

    int key = ' ';

    do
    {
        //Capturing Detection data from the camera
        data =  PAL::GrabTrackingData();

        cv::Mat display = data[0].left;
        int num_of_persons = data[0].trackingData[PAL::States::OK].size();

        char text[128];
        for(int i=0; i<num_of_persons; i++)
        {
            int x1,y1,x2,y2;
            x1 = (int)data[0].trackingData[PAL::States::OK][i].boxes.x1;
            y1 = (int)data[0].trackingData[PAL::States::OK][i].boxes.y1;
            x2 = (int)data[0].trackingData[PAL::States::OK][i].boxes.x2;
            y2 = (int)data[0].trackingData[PAL::States::OK][i].boxes.y2;

            float x3D, y3D, z3D, depth_value;
            x3D = data[0].trackingData[PAL::States::OK][i].locations_3d.x;
            y3D = data[0].trackingData[PAL::States::OK][i].locations_3d.y;
            z3D = data[0].trackingData[PAL::States::OK][i].locations_3d.z;

            //get the depth of each person from the camera
            depth_value = sqrt(x3D*x3D + y3D*y3D);
            sprintf(text, "Depth:%.2fm", depth_value);

            //depth value calculated is in meters and the threshold entered is in centimeters
            bool safe = depth_value*100 > safe_distance;

            if(safe)
                //Drawing GREEN box indicating the person is socially distant
                cv::rectangle(display, cv::Rect(x1, y1, x2, y2), cv::Scalar(0,255,0),2);
            else
                //Drawing RED box indicating the person is not socially distant 
                cv::rectangle(display, cv::Rect(x1, y1, x2, y2), cv::Scalar(0,0,255),2);

            setLabel(display, text, Point(x1, y1), CV_RGB(255,0,0));
        }
        
        //Display the stereo images
        imshow( "PAL Safe Zone Distance", display);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;

        if (key == 'q' || key == 'Q')
        {
            detection_threshold += 0.1;
            if(detection_threshold>1)
            {
                detection_threshold = 1;
                std::cout << "Max threshold (1.0) reached" << std::endl;
            }
            PAL::SetDetectionModeThreshold(detection_threshold, class_id);
        }
        if (key == 'a' || key == 'A')
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
