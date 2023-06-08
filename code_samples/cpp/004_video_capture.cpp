/*

CODE SAMPLE # 004: PAL Video Capture
This code will grab the left & depth panorama and display in a window using opencv

>>>>>> Compile this code using the following command....

./compile.sh 004_video_capture.cpp

>>>>>> Execute the binary file by typing the following command...

./004_video_capture.out

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
    namedWindow( "PAL Range Scan", WINDOW_AUTOSIZE);

    cout<<"Press ESC to close the window."<<endl;
    cout<<"Press v/V to toggle vertical flip property"<<endl;
    cout<<"Press f/F to toggle filter spots property"<<endl;
    cout<<"Press C to capture a single frame into a PNG file."<<endl;
    cout<<"Press B to begin the video capture."<<endl;
    cout<<"Press E to end the video capture."<<endl;

    int image_count = 0;
    int video_count = 0;
    bool record = false;

    cv::VideoWriter video;

    std::vector<PAL::Data::ODOA_Data> data;
    
    int key = ' ';
    
    do
    {
        //Capturing Depth data from the camera
        data =  PAL::GrabRangeScanData();    

        Mat left = data[0].left;
        Mat depth;
        if(properties.raw_depth)
            depth = data[0].fused_depth.clone();
        else
            depth = data[0].distance.clone();    

        //convert the 32FC1 depth map to 8UC3 for visualisation
        depth.convertTo(depth, CV_8UC1);
        cvtColor(depth, depth, cv::COLOR_GRAY2BGR);

        Mat display;

        //Vertical concatenation of colored left and depth into the final output
        vconcat(left, depth, display);

        //Display the depth with colored left panorama
        imshow( "PAL Video Capture", display);  

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
        
        //capture an image using imwrite
        if(key == 'C' || key == 'c') 
        {
            char image_filename[128];
            sprintf(image_filename, "image_%d.png", ++image_count);
            imwrite(image_filename, display);

            cout<<"The current frame is saved as " << image_filename <<endl;
        }

        //capture a video using VideoWriter
        if(key == 'B' || key == 'b')
        {
            cout<<"Opening the video"<<endl;

            char video_filename[128];
            sprintf(video_filename, "pal_video_%d.avi", ++video_count);
            cv::Size size = cv::Size(display.cols, display.rows);
            int fps = 15;

            video = cv::VideoWriter(video_filename, cv::VideoWriter::fourcc('X','V','I','D'), fps, size);
            
            record = true;
        }
        if (key == 'E' || key == 'e')
        {
            if(record)
            {
                record = false;            
                cout<<"Releasing the video"<<endl;
                video.release();
            }
        }
        if (key == 27 || data[0].camera_changed)
        {
            if(record)
            {
                record = false;            
                cout<<"Releasing the video"<<endl;
                video.release();
            }
        }
        if(record)
        { 
            video.write(display);
        }
    }
    //27 = esc key. Run the loop until the ESC key is pressed and camera is not changed
    while(key != 27 && !data[0].camera_changed);

    cout<<"exiting the application"<<endl;
    PAL::Destroy();
   
    return 0;
}
