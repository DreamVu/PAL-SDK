/*

CODE SAMPLE # 014: Object Tracking With GPIO compatibilty
This code will grab the 360 rgb data, do object tracking and toggle GPIO pins of the Nvidia board.


>>>>>> Compile this code using the following command....

./compile.sh 014_object_tracking_GPIO.cpp

>>>>>> Execute the binary file by typing the following command...

./014_object_tracking_GPIO.out

>>>>>> KEYBOARD CONTROLS:

    Press CTRL+C to close the application.
*/


# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"
#include "TimeLogger.h"
#include <time.h>
#include <unistd.h>
#include <JetsonGPIO.h>
#include<sys/time.h>
#include <chrono>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <iomanip>

using namespace cv;
using namespace std;

int person_detected = 0;
static bool g_bExit = false;
void signalHandler(int signum)
{	
	g_bExit = true;
}

// camera and detection pins
int camera_pin = 15 ;
int detection_pin = 7;
int camera_pin_1 = 29;
int camera_pin_2 = 31;
int camera_pin_3 = 33;


int main( int argc, char** argv )
{
	signal(SIGINT, signalHandler);
	//setting mode to GPIO board
	GPIO::setmode(GPIO::BOARD);
	GPIO::setwarnings(false);
	//setting camera pins active 
	GPIO::setup(camera_pin, GPIO::OUT, GPIO::LOW);
	GPIO::setup(camera_pin_1, GPIO::OUT, GPIO::LOW);
	GPIO::setup(camera_pin_2, GPIO::OUT, GPIO::LOW);
	GPIO::setup(camera_pin_3, GPIO::OUT, GPIO::LOW);
	
	//setting detection pin inactive by default
	GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);
	
    //Select the Model to use in Tracking. To be set before Init call.
    PAL::SetInitTrackingModel(PAL::Tracking_Model::MODEL_0);

    int width, height;
    std::vector<int> camera_indexes{5};
    PAL::Mode def_mode = PAL::Mode::LASER_SCAN;

	GPIO::output(camera_pin, 1);
	GPIO::output(camera_pin_1, 1);
	GPIO::output(camera_pin_2, 1);
	GPIO::output(camera_pin_3, 1);
	GPIO::output(detection_pin, 1);

	PAL::DisableModels(true);
	PAL::SyncronizeInputs(true);
	
    //Start the PAL application
    if (PAL::Init(width, height, camera_indexes, &def_mode) != PAL::SUCCESS) //Connect to the PAL camera
    {
        cout<<"Init failed"<<endl;
        return 1;
    }
    
    GPIO::output(camera_pin, 0);
    GPIO::output(camera_pin_1, 0);
    GPIO::output(camera_pin_2, 0);
    GPIO::output(camera_pin_3, 0);
    GPIO::output(detection_pin, 1);

    //Select which mode you want to run the application in.
    PAL::SetAPIMode(PAL::API_Mode::TRACKING);
    usleep(1000000);

    PAL::CameraProperties data;
    PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &data);

    if(ack_load != PAL::SUCCESS)
    {
        cout<<"Error Loading settings! Loading default values."<<endl;
    }
	
	PAL::CameraProperties prop;
	prop.exposure = 1000;
	prop.auto_exposure = 1;
	prop.auto_exposure_method = 1;

	unsigned long int flags = PAL::EXPOSURE ;
	flags = flags | PAL::AUTO_EXPOSURE | PAL::AUTO_EXPOSURE_METHOD ;
	PAL::SetCameraProperties(&prop, &flags);

    int tracking_mode = PAL::Tracking_Mode::PEOPLE_TRACKING;
    int success = PAL::SetModeInTracking(tracking_mode);

	bool enableDepth = false;
	bool enable3Dlocation = false;
	bool bDetectionPinActive = false;
	extern bool camera_disconnected;
	
	std::cout << "Press CTRL+C to close the application." << std::endl;

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

    printf("exiting the application\n");
    GPIO::setup(camera_pin, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(camera_pin_1, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(camera_pin_2, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(camera_pin_3, GPIO::OUT, GPIO::HIGH);
	GPIO::setup(detection_pin, GPIO::OUT, GPIO::HIGH);
	GPIO::cleanup();
	
    PAL::Destroy();
   
    return 0;
}

