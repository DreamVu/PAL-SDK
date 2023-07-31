/*

CODE SAMPLE # 005: PAL Camera Properties
This code will grab the left & depth panorama and display in a window using opencv

>>>>>> Compile this code using the following command....

./compile.sh 005_camera_properties.cpp

>>>>>> Execute the binary file by typing the following command...

./005_camera_properties.out

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
    PAL::SetAPIMode(PAL::API_Mode::DEPTH);
    
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

    // Create a window for display.
    namedWindow( "PAL Camera Properties", WINDOW_AUTOSIZE);

    cout<<"Press ESC to close the window."<<endl;
    cout<<"X & M keys increase and decrease the BRIGHTNESS respectively."<<endl;
    cout<<"W & S keys increase and decrease the CONTRAST respectively."<<endl;
    cout<<"E & D keys increase and decrease the SATURATION respectively."<<endl;
    cout<<"R & Z keys increase and decrease the GAMMA respectively."<<endl;
    cout<<"T & G keys increase and decrease the GAIN respectively."<<endl;
    cout<<"Y & H keys increase and decrease the WHITE BALANCE TEMPERATURE respectively."<<endl;
    cout<<"U & J keys increase and decrease the SHARPNESS respectively."<<endl;
    cout<<"I & K keys increase and decrease the EXPOSURE respectively."<<endl;
    cout<<"B & Q keys increase and decrease the FOCUS respectively."<<endl;
    cout<<", & . keys increase and decrease the HUE respectively."<<endl;
    cout<<"O key toggles AUTO WHITE BALANCE property."<<endl;
    cout<<"P key toggles AUTO EXPOSURE property."<<endl;
    cout<<"A key toggles AUTO FOCUS property."<<endl;
    cout<<"C key saves the current left+depth panorama image to a numbered file."<<endl;
    cout<<"N key saves the current camera properties to a file."<<endl;
    cout<<"L key loads the camera properties from the saved file."<<endl;
    cout<<"V key toggles VERTICAL FLIP property."<<endl;
    cout<<"F key toggles FILTER SPOTS property."<<endl;
    cout<<"Space key sets the default properties."<<endl;

    int frame = 0;
    PAL::CameraPropertyValues cpv;

    std::vector<PAL::Data::ODOA_Data> data;

    int key = ' ';
    
    do
    {
        //Capturing Depth data from the camera
        data =  PAL::GrabRangeScanData();    
        
        Mat left = data[0].left;
        Mat depth;
        if(properties.raw_depth)
            depth = data[0].raw_depth.clone();
        else
            depth = data[0].depth.clone();    

        //convert the 32FC1 depth map to 8UC3 for visualisation
        depth.convertTo(depth, CV_8UC1);
        cvtColor(depth, depth, cv::COLOR_GRAY2BGR);

        Mat display;

        //Vertical concatenation of rgb and depth into the final output
        vconcat(left, depth, display);

        //Display the depth with rgb panorama
        imshow( "PAL Camera Properties", display);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
        
        unsigned long int flags = 0;

        switch (key)
        {   
            case ' ':   
            // set default properties
                PAL::SetDefaultCameraProperties(&properties);
            break;
            // increase brightness
            case 'x':   
            case 'X':
                properties.brightness += 1;
                if(properties.brightness > cpv.MAX_BRIGHTNESS) properties.brightness = cpv.MAX_BRIGHTNESS; 
                flags |= PAL::BRIGHTNESS;
            break;
            // decrease brightness
            case 'm':  
            case 'M':
                properties.brightness -= 1;
                if(properties.brightness < cpv.MIN_BRIGHTNESS) properties.brightness = cpv.MIN_BRIGHTNESS;
                flags |= PAL::BRIGHTNESS;
            break;
            // increase contrast 
            case 'w':  
            case 'W':
                properties.contrast += 1;
                if(properties.contrast > cpv.MAX_CONTRAST) properties.contrast = cpv.MAX_CONTRAST; 
                flags |= PAL::CONTRAST;
            break;
            // decrease contrast
            case 's':  
            case 'S':
                properties.contrast -= 1;
                if(properties.contrast < cpv.MIN_CONTRAST) properties.contrast = cpv.MIN_CONTRAST;
                flags |= PAL::CONTRAST;
            break;
            // increase saturation
            case 'e':   
            case 'E':
                properties.saturation += 1;
                if(properties.saturation > cpv.MAX_SATURATION) properties.saturation = cpv.MAX_SATURATION; 
                flags |= PAL::SATURATION;
            break;
            // decrease saturation
            case 'd':  
            case 'D':
                properties.saturation -= 1;
                if(properties.saturation < cpv.MIN_SATURATION) properties.saturation = cpv.MIN_SATURATION;
                flags |= PAL::SATURATION;
            break;
             // increase gamma
            case 'r': 
            case 'R':
                properties.gamma += 10;
                if(properties.gamma > cpv.MAX_GAMMA) properties.gamma = cpv.MAX_GAMMA; 
                flags |= PAL::GAMMA;
            break;
            // decrease gamma
            case 'z':   
            case 'Z':
                properties.gamma -= 10;
                if(properties.gamma < cpv.MIN_GAMMA) properties.gamma = cpv.MIN_GAMMA;
                flags |= PAL::GAMMA;
            break;
            // increase gain
            case 't':   
            case 'T':
                properties.gain += 1;
                if(properties.gain > cpv.MAX_GAIN) properties.gain = cpv.MAX_GAIN;
                flags |= PAL::GAIN;
            break;
            // decrease gain
            case 'g':   
            case 'G':
                properties.gain -= 1;
                if(properties.gain < cpv.MIN_GAIN) properties.gain = cpv.MIN_GAIN;
                flags |= PAL::GAIN;
            break;
            // increase white balance temperature
            case 'y':  
            case 'Y':
                properties.white_bal_temp += 200;
                if(properties.white_bal_temp > cpv.MAX_WHITE_BAL_TEMP) properties.white_bal_temp = cpv.MAX_WHITE_BAL_TEMP;
                flags |= PAL::WHITE_BAL_TEMP;
            break;
            // decrease white balance temperature
            case 'h':  
            case 'H':
                properties.white_bal_temp -= 200;
                if(properties.white_bal_temp < cpv.MIN_WHITE_BAL_TEMP) properties.white_bal_temp = cpv.MIN_WHITE_BAL_TEMP;
                flags |= PAL::WHITE_BAL_TEMP;
            break;
            // increase sharpness 
            case 'u':   
            case 'U':
                properties.sharpness += 1;
                if(properties.sharpness > cpv.MAX_SHARPNESS) properties.sharpness = cpv.MAX_SHARPNESS;
                flags |= PAL::SHARPNESS;
            break;
            // decrease sharpness
            case 'j':   
            case 'J':
                properties.sharpness -= 1;
                if(properties.sharpness < cpv.MIN_SHARPNESS) properties.sharpness = cpv.MIN_SHARPNESS;
                flags |= PAL::SHARPNESS;
            break;
            // increase exposure
            case 'i':   
            case 'I':
                properties.exposure += 5;
                if(properties.exposure > cpv.MAX_EXPOSURE) properties.exposure = cpv.MAX_EXPOSURE;
                flags |= PAL::EXPOSURE;
            break;
            // decrease exposure 
            case 'k':  
            case 'K':
                properties.exposure -= 5;
                if(properties.exposure < cpv.MIN_EXPOSURE) properties.exposure = cpv.MIN_EXPOSURE;
                flags |= PAL::EXPOSURE;
            break;
	          // increase focus
            case 'b':   
            case 'B':
                properties.focus += 5;
                if(properties.focus > cpv.MAX_FOCUS) properties.focus = cpv.MAX_FOCUS;
                flags |= PAL::FOCUS;
            break;
            // decrease focus 
            case 'q':  
            case 'Q':
                properties.focus -= 5;
                if(properties.focus < cpv.MIN_FOCUS) properties.focus = cpv.MIN_FOCUS;
                flags |= PAL::FOCUS;
            break;            
			      // increase hue
            case ',':   
 		        case '<': 
                properties.hue += 5;
                if(properties.hue > cpv.MAX_HUE) properties.hue = cpv.MAX_HUE;
                flags |= PAL::HUE;
            break;
            // decrease hue 
            case '.':  
            case '>':
                properties.hue -= 5;
                if(properties.hue < cpv.MIN_HUE) properties.hue = cpv.MIN_HUE;
                flags |= PAL::HUE;
            break;      		
			      // Toggle auto white balance temperature
            case 'o':  
            case 'O':
                properties.auto_white_bal = !properties.auto_white_bal;
                flags |= PAL::AUTO_WHITE_BAL;
            break;  
            // Toggle auto exposure                   
            case 'p':   
            case 'P':
                properties.auto_gain = !properties.auto_gain;
                flags |= PAL::AUTO_GAIN;
            break;
			      // Toggle auto focus                   
            case 'a':   
            case 'A':
                properties.auto_focus = !properties.auto_focus;
                flags |= PAL::AUTO_FOCUS;
            break;
            // Saves current left+depth image to disk as a .png file    
            case 'c':  
            case 'C':
                char fileName[128];
                sprintf(fileName,"./pal_image_%03d.png", frame++);
                cv::imwrite(fileName, display);
            break;
            // Saves camera properties to file           
            case 'n': 
            case 'N':
                PAL::SaveProperties("properties.txt");
                cout<<">>>>>> SAVED THE PROPERTIES >>>>"<<endl;
            break;
            // Loads camera properties from file
            case 'l':  
            case 'L':
                PAL::LoadProperties("properties.txt", &properties);
                break;
            //Toogle vertical flip     
            case 'V':  
            case 'v':
                properties.vertical_flip = !properties.vertical_flip;
                flags |= PAL::VERTICAL_FLIP;
            break; 
            // Toggle Filter spots
            case 'F':  
            case 'f':
                properties.filter_spots = !properties.filter_spots;
                flags |= PAL::FILTER_SPOTS;
            break;        
        }
        
        if(flags != 0)
        {
            PAL::SetCameraProperties(&properties, &flags);
            cout<<"Camera Properties...."<<endl;
            cout<<"brightness         = "<<properties.brightness<<endl;
            cout<<"contrast           = "<<properties.contrast<<endl;
            cout<<"saturation         = "<<properties.saturation<<endl;
            cout<<"gamma              = "<<properties.gamma<<endl;
            cout<<"gain               = "<<properties.gain<<endl;
            cout<<"white_bal_temp     = "<<properties.white_bal_temp<<endl;
            cout<<"sharpness          = "<<properties.sharpness<<endl;
            cout<<"exposure           = "<<properties.exposure<<endl;
            cout<<"focus              = "<<properties.focus<<endl;
            cout<<"hue                = "<<properties.hue<<endl;
            cout<<"auto_white_bal     = "<<properties.auto_white_bal<<endl;
            cout<<"auto_gain          = "<<properties.auto_gain<<endl;
            cout<<"auto_focus         = "<<properties.auto_focus<<endl<<endl;
        }
    }
    //27 = esc key. Run the loop until the ESC key is pressed and camera is not changed
    while(key != 27 && !data[0].camera_changed);

    cout<<"exiting the application"<<endl;
    PAL::Destroy();

    return 0;
}
