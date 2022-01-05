/*

CODE SAMPLE # 001: Basic stereo panorama
This code will grab the basic stereo panoramas (left and right images) and would be displayed in a window using opencv


>>>>>> Compile this code using the following command....


g++ 001_stereo_panorama.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -g  -o 001_stereo_panorama.out -I../include/ -lv4l2 -lpthread


>>>>>> Execute the binary file by typing the following command...


./001_stereo_panorama.out


>>>>>> KEYBOARD CONTROLS:

       ESC key closes the window
       

*/


# include <stdio.h>
# include <unistd.h>
# include <sys/types.h>
# include <opencv2/opencv.hpp>

# include "PAL.h"

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
    int camera_index=1;
    cout << "Please enter you camera index(default is 1) and press Enter: ";
    cin >> camera_index;
    namedWindow( "PAL window", WINDOW_NORMAL ); // Create a window for display.
    
    int width=-1, height=-1;
    if(PAL::Init(width, height,camera_index) != PAL::SUCCESS) //Connect to the PAL camera
    {
        printf("Init failed\n");
        return 1;
    }
    
    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed at one fourth their original resolution.
    //Since the panoramas are vertically stacked, the window height should be twice of 1/4th height
    resizeWindow("PAL window", width/4, (height/4)*2);
    
    int key = ' ';
    
    printf("Press ESC to close the window.\n");

	printf("The image resolution is .... %dx%d\n", width, height);


    
    Mat output = cv::Mat::zeros(height, width, CV_8UC3);
    
    //Display the concatenated image
    imshow( "PAL window", output);
    
    //27 = esc key. Run the loop until the ESC key is pressed
    while(key != 27)
    {
        PAL::Image left, right;
        PAL::GrabFrames(&left, &right, 0);
        
        //Convert PAL::Image to Mat
        Mat l = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
        Mat r = Mat(right.rows, right.cols, CV_8UC3, right.Raw.u8_data);
        
        //Vertical concatenation of left and right images
        vconcat(l, r, output);
        
        //Display the concatenated image
        imshow( "PAL window", output);  
        
        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
    }

    printf("exiting the application\n");
    PAL::Destroy();
    return 0;
}

