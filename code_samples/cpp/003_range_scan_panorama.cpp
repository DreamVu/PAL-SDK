/*

CODE SAMPLE # 003: Range scan panorama
This code will grab the left panorama with range scan overlayed on it and would be displayed in a window using opencv

>>>>>> Compile this code using the following command....

./compile.sh 003_range_scan_panorama.cpp

>>>>>> Execute the binary file by typing the following command...

./003_range_scan_panorama.out
        
*/

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "PAL.h"

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
    PAL::SetAPIMode(PAL::API_Mode::RANGE_SCAN);
    
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
        
    // Create a window for display.
    namedWindow( "PAL Range Scan", WINDOW_AUTOSIZE);

    cout<<"Press ESC to close the window."<<endl;
    cout<<"Press v/V to toggle vertical flip property"<<endl;
    
    std::vector<PAL::Data::ODOA_Data> data;
    
    int key = ' ';

    do
    {
        //Capturing Range Scan data from the camera
        data =  PAL::GrabRangeScanData();    
        
        //Display the overlayed image
        imshow( "PAL Range Scan", data[0].scan_overlay_left);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
        
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
