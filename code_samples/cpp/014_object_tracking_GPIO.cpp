/*

CODE SAMPLE # 014: Object Tracking With GPIO compatibilty
This code will grab the 360 rgb data, do object tracking and toggle GPIO pins of the Nvidia/DreamVu board.

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

//Pins for Nvidia dev board
int detection_pin_nvidia = 7;
std::vector<int> camera_pins_nvidia = {15, 29, 31, 33};

//Pins for DreamVu board
int detection_pin_dreamvu = 20;
std::vector<int> camera_pins_dreamvu = {12, 13};


//Used to handle ctrl + c
static bool g_bExit = false;
void signalHandler(int signum)
{    
    g_bExit = true;
}

void get_default_pins(int board_type, int &detection_pin, std::vector<int> &camera_pins)
{
    if(board_type == 0) //nvidia dev board
    {
        detection_pin = detection_pin_nvidia;
        camera_pins = camera_pins_nvidia;
    }
    else if(board_type == 1) //dreamvu boards
    {
        detection_pin = detection_pin_dreamvu;
        camera_pins = camera_pins_dreamvu;
    }
}

int main( int argc, char** argv )
{
    //Handle system signals
    signal(SIGINT, signalHandler);

    //Setting mode to GPIO board
    GPIO::setmode(GPIO::BOARD);
    GPIO::setwarnings(false);

    //Carrier board detection. 0 for nvidia dev board, 1 for dreamvu boards. 
    //TO be called after GPIO::setmode()
    int board_type = GPIO::get_board_type();

    // camera and detection pins
    int detection_pin;
    std::vector<int> camera_pins;

    //Get board dependent default pins used for camera
    get_default_pins(board_type, detection_pin, camera_pins);

    /*If you want to want to use custom pins, then please find the
      correct pins for your board and assign them in the following fashion */
    //camera_pins = std::vector<int>({pin_1, pin_2, pin_3, ...});

    //Pins used for the camera
    std::cout << "The following pins are used for this GPIO application:" << std::endl;

    std::cout << "Detection pin is: " << detection_pin << std::endl;
    std::cout << "Camera pins are:";
    for (auto camera_pin : camera_pins)
    {
        std::cout << " " << camera_pin; 
    }
    std::cout << std::endl;

    //Setting camera pins active 
    for (auto camera_pin : camera_pins)
    {
        GPIO::setup(camera_pin, GPIO::OUT, GPIO::LOW);
    }
    
    //Setting detection pin inactive by default
    GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);

    //Setting pins to camera not found state before initialising the camera
    for (auto camera_pin : camera_pins)
    {
        GPIO::output(camera_pin, 1);
    }
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
    for (auto camera_pin : camera_pins)
    {
        GPIO::output(camera_pin, 0);
    }
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
            for (auto camera_pin : camera_pins)
            {
                GPIO::output(camera_pin, 1);
            }
            GPIO::output(detection_pin, 1);
            bDetectionPinActive = false; 
            person_detected = false;         
        }
        else
        {    
            for (auto camera_pin : camera_pins)
            {
                GPIO::output(camera_pin, 0);
            }   
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
    for (auto camera_pin : camera_pins)
    {
        GPIO::setup(camera_pin, GPIO::OUT, GPIO::HIGH);
    }
    GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);
    GPIO::cleanup();
    
    PAL::Destroy();
   
    return 0;
}
