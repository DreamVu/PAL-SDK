/*

CODE SAMPLE # 001: Stereo Panorama
This code will grab the left & right panorama and display in a window using opencv

>>>>>> Compile this code using the following command....

./compile.sh 001_stereo_panorama.cpp

>>>>>> Execute the binary file by typing the following command...

./001_stereo_panorama.out

*/

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "PAL.h"
#include <X11/Xlib.h>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
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
    PAL::SetAPIMode(PAL::API_Mode::STEREO);
    
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

    //Setting display size to fullscreen
    Display* disp = XOpenDisplay(NULL);
    Screen*  scrn = DefaultScreenOfDisplay(disp);
    int sc_height = scrn->height;
    int sc_width  = scrn->width;
        
    // Create a window for display.
    namedWindow( "PAL Stereo Panorama", WINDOW_NORMAL ); 
    resizeWindow("PAL Stereo Panorama", sc_width-60, sc_height-60);

    cout<<"Press ESC to close the window."<<endl;
    cout<<"Press f/F to toggle filter spots property"<<endl;
    cout<<"Press v/V to toggle vertical flip property"<<endl;
    
    std::vector<PAL::Data::Stereo> data;
    
    int key = ' ';

    do
    {
        //Capturing Stereo data from the camera
        data =  PAL::GetStereoData();

        cv::Mat display;
        vconcat(data[0].stereo_left, data[0].stereo_right, display);
        
        //Display the stereo images
        imshow( "PAL Stereo Panorama", display);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
        
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
    }
    //27 = esc key. Run the loop until the ESC key is pressed and camera is not changed
    while(key != 27 && !data[0].camera_changed);

    cout<<"exiting the application"<<endl;
    PAL::Destroy();
   
    return 0;
}
