/*

CODE SAMPLE # 003: Video/image capture

This code will save the panoramas into still images or videos - depending on the user input


>>>>>> Compile this code using the following command....


g++ 003_video_capture.cpp ../lib/PAL.a `pkg-config --libs --cflags opencv`    -O3 -o 003_video_capture -I../include/ -lv4l2


>>>>>> Execute the binary file by typing the following command...


./003_video_capture


>>>>>> KEYBOARD CONTROLS:

       'B' key begins the video capture
       'E' key ends the video capture
       'C' key captures a single frame into a PNG file
       ESC key closes the window
       

*/



# include <stdio.h>

# include <opencv2/opencv.hpp>
# include <opencv2/highgui.hpp>

# include "PAL.h"


using namespace cv;
using namespace std;




int main( int argc, char** argv )
{
    int camera_index=1;
    cout << "Please enter you camera index(default is 1) and press Enter: ";
    cin >> camera_index;
    namedWindow( "PAL window", WINDOW_NORMAL );
    
    int width, height;
    if(PAL::Init(width, height,camera_index) != PAL::SUCCESS)
    {
        printf("Init failed\n");
        return 1;
    }
    
    int window_width = width/4;
    int window_height = (height/4)*2;
    resizeWindow("PAL window", window_width, window_height);
    int key = ' ';
    bool record = false;
    bool closed = false;
    
    printf("Press ESC to close the window\n");
    printf("Press C to capture a single frame into a PNG file\n");
    printf("Press B to begin the video capture\n");
    printf("Press E to end the video capture\n");
    
    
    cv::VideoWriter video;
    
    while(!closed)
    {
        PAL::Image left, right, depth, disparity;
        PAL::GrabFrames(&left, &right, &depth, &disparity, false);
        
        Mat temp, output;
        Mat l = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
        Mat d = Mat(disparity.rows, depth.cols, CV_16SC1, disparity.Raw.u8_data);
	d.convertTo(d, CV_8UC1);
        cvtColor(d, d, cv::COLOR_GRAY2BGR);
        
        vconcat(l, d, output);
        imshow( "PAL window", output);  
        key = waitKey(1) & 255;
        
        if(key == 'C' || key == 'c')
        {
            imwrite("image.png", output);
            printf("The current frame is saved as image.png\n");
        }
        else if(key == 'B' || key == 'b')
        {
            cv::Size size = cv::Size(output.cols, output.rows);
            int fps = 10;
            printf("Opening the video\n");
            video = cv::VideoWriter("pal_video.avi",cv::VideoWriter::fourcc('X','V','I','D'), fps, size);
            record = true;
        }
        else if (key == 'E' || key == 'e' || key == 27)
        {
            closed = true;
            if(record)
            {
                record = false;            
                printf("Releasing the video \n");
                video.release();
            }
        }
        
        if(record)
        { 
            video.write(output);
        }
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}

