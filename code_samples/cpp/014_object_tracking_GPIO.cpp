/*

CODE SAMPLE # 014: Object Tracking With GPIO compatibilty
This code will grab the 360 rgb data, do object tracking and toggle GPIO pins of the Nvidia board.

>>>>>> Compile this code using the following command....

./compile.sh 014_object_tracking_GPIO.cpp

>>>>>> Execute the binary file by typing the following command...

./014_object_tracking_GPIO.out

*/

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "PAL.h"
#include <JetsonGPIO.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iomanip>
#include <csignal>

using namespace cv;
using namespace std;

// camera and detection pins
int camera_pin = 15;
int detection_pin = 7;
int camera_pin_1 = 29;
int camera_pin_2 = 31;
int camera_pin_3 = 33;

static bool g_bExit = false;
void signalHandler(int signum)
{    
    g_bExit = true;
}

int main( int argc, char** argv )
{
    //Handle system signals
    signal(SIGINT, signalHandler);

    //Setting mode to GPIO board
    GPIO::setmode(GPIO::BOARD);
    GPIO::setwarnings(false);

    //Setting camera pins active 
    GPIO::setup(camera_pin, GPIO::OUT, GPIO::LOW);
    GPIO::setup(camera_pin_1, GPIO::OUT, GPIO::LOW);
    GPIO::setup(camera_pin_2, GPIO::OUT, GPIO::LOW);
    GPIO::setup(camera_pin_3, GPIO::OUT, GPIO::LOW);
    
    //Setting detection pin inactive by default
    GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);

    //Setting pins to camera not found state before initialising the camera
    GPIO::output(camera_pin, 1);
    GPIO::output(camera_pin_1, 1);
    GPIO::output(camera_pin_2, 1);
    GPIO::output(camera_pin_3, 1);
    GPIO::output(detection_pin, 1);

    PAL::SyncronizeInputs(true);
    
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
    
    //Setting pins to camera found state after initialising the camera
    GPIO::output(camera_pin, 0);
    GPIO::output(camera_pin_1, 0);
    GPIO::output(camera_pin_2, 0);
    GPIO::output(camera_pin_3, 0);
    GPIO::output(detection_pin, 1);

    //Select which mode you want to run the application in.
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

    //Changing default properties to ideal values for this application
    PAL::CameraProperties prop;
    prop.exposure = 1000;
    prop.auto_gain = 1;
    prop.auto_exposure_method = 1;

    unsigned long int flags = PAL::EXPOSURE ;
    flags = flags | PAL::AUTO_GAIN | PAL::AUTO_EXPOSURE_METHOD ;
    PAL::SetCameraProperties(&prop, &flags);

    //Set depth detection mode
    PAL::SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_OFF);

    int tracking_mode = PAL::Tracking_Mode::PEOPLE_TRACKING;
    int success = PAL::SetModeInTracking(tracking_mode);

    bool bDetectionPinActive = false;
    extern bool camera_disconnected;
    
    std::cout << "Press CTRL+C to close the application." << std::endl;

    int person_detected = 0;

    while(!g_bExit)
    {
        if(camera_disconnected)
        {
            GPIO::output(camera_pin, 1);
            GPIO::output(camera_pin_1, 1);
            GPIO::output(camera_pin_2, 1);
            GPIO::output(camera_pin_3, 1);
            GPIO::output(detection_pin, 1);
            bDetectionPinActive = false; 
            person_detected = false;         
        }
        else
        {    
            GPIO::output(camera_pin, 0);
            GPIO::output(camera_pin_1, 0);
            GPIO::output(camera_pin_2, 0);
            GPIO::output(camera_pin_3, 0);    
        }    

        std::vector<PAL::Data::TrackingResults> data;
        data =  PAL::GrabTrackingData();    
        if(data[0].camera_changed)
        {
            //exiting application when camera is changed
            break;
        }
        person_detected = data[0].trackingData[PAL::States::OK].size();

        if(!camera_disconnected)
        {
            if(person_detected)
            {
                if(!bDetectionPinActive)
                {
                    cout << "Setting GPIO detection pin active" << endl;     
                    GPIO::output(detection_pin, 0);
                    bDetectionPinActive = true;
                }
            }
            else
            {
                if(bDetectionPinActive)
                {
                    cout << "Setting GPIO detection pin inactive" << endl;
                    GPIO::output(detection_pin, 1);
                    bDetectionPinActive = false;
                }
            }
        }
    }

    cout << "exiting the application" << endl;

    //Setting pins to camera disconnected and no detection state
    GPIO::setup(camera_pin, GPIO::OUT, GPIO::HIGH);
    GPIO::setup(camera_pin_1, GPIO::OUT, GPIO::HIGH);
    GPIO::setup(camera_pin_2, GPIO::OUT, GPIO::HIGH);
    GPIO::setup(camera_pin_3, GPIO::OUT, GPIO::HIGH);
    GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);
    GPIO::cleanup();
    
    PAL::Destroy();
   
    return 0;
}
