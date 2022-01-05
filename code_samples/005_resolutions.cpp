/*

CODE SAMPLE # 005: Switching Resolutions
This code sample allows users to access data proccessed at different resolutions.
Specifically, left, right, disparity and depth. 
Users can also get pixel specific RGB, disparity and depth information on mouse click.

>>>>>> Compile this code using the following command....


g++ 005_resolutions.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv` -g  -o 005_resolutions.out -I../include/ -lv4l2 -lpthread -std=c++11


>>>>>> Execute the binary file by typing the following command...


./005_resolutions.out


>>>>>> KEYBOARD CONTROLS:

       ESC key closes the window
	   'n' key toggles the disparity normalization
	   '1' key uses the full resolution
	   '2' key uses only half the resolution
	   '3' key uses only one third resolution
	   
>>>>>> MOUSE CONTROLS:

    Click on any point in any of the three panoramas (left/right/disparity)
    The following values will be displayed on the screen...
    --> pixel location
    --> RGB value of the pixel
    --> Disparity at that pixel
    --> Depth at that pixel
    
    And, the scaled depth data is saved as an image with the name "depth_on_click.png"
    

*/




# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"

using namespace cv;
using namespace std;

Mat l, r, d, depthMat;

void my_mouse_callback( int event, int x, int y, int flags, void* param ) 
{
    float* depth = (float*)depthMat.data;
    uchar* l_rgb = l.data;
    uchar* r_rgb = r.data;
    
    
    if(event==CV_EVENT_LBUTTONDOWN)
    {
    
    printf("\n\nOriginal (x=%d, y=%d)\n", x, y);
       
    
    x %= l.cols;
    y %= l.rows;
    
    int index = (x + y*l.cols);
    l_rgb += index*3;
    r_rgb += index*3;
    depth += index;
     
        PAL::Image dummy;        
        dummy.Create(l.cols, l.rows, 1, 1);
        unsigned char* temp1 = dummy.Raw.u8_data;
        float* temp2 = (float*)depthMat.data;
        for(int i = 0; i < l.rows; i++)
        {
            for(int j = 0; j < l.cols; j++)
            {
                int g = (int) *temp2++;               
                if(g < 0) g = 0;
                if(g > 255) g = 255;
                *temp1++ = (uchar)g;
            }
        } 
        Mat grey = Mat(l.rows, l.cols, CV_8UC1, dummy.Raw.data);
        imwrite("depth_on_click.png", grey);
        
        printf("Displaying the details at (x=%d, y=%d)\n", x, y);
        printf("RGB @ left = (%3d, %3d, %3d)\n", l_rgb[0], l_rgb[1], l_rgb[2]);
        printf("RGB @ right = (%3d, %3d, %3d)\n", r_rgb[0], r_rgb[1], r_rgb[2]);
        printf("Depth_float = %f\n",depth[0]); 
        printf("Depth_uchar = %d\n",dummy.Raw.u8_data[index]); 
        printf("\n"); 
        
        dummy.Destroy(); 
    }
    
}

void saveinBin(Mat img, string filename)
{
    FILE *f = fopen(filename.c_str(), "wb");
    fwrite(img.data, sizeof(float), img.rows * img.cols, f);
    fclose(f);
}

int main( int argc, char** argv )
{

    namedWindow( "PAL window", WINDOW_NORMAL ); // Create a window for display.
    
    
    setMouseCallback( "PAL window", my_mouse_callback, 0);
    
    int width, height;
    if(PAL::Init(width, height,-1) != PAL::SUCCESS) //Connect to the PAL camera
    {
        printf("Init failed\n");
        return 1;
    }

    printf("The image resolution is .... %dx%d\n", width, height);
    
    
    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed at quarter their original resolution.
    //Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
    resizeWindow("PAL window", width/4, (height/4)*3);
    
    int key = ' ';
    
    printf("Press ESC to close the window\n");    
    printf("Press N to toggle the disparity normalization\n");
	printf("Press '1' key to use the full resolution\n");
	printf("Press '2' key to use only half the resolution\n");
	printf("Press '3' key to use only one third resolution\n");  
	printf("Click on any pixel to get the RGBD data, and save the depth image\n");
    
    
    //27 = esc key. Run the loop until the ESC key is pressed
    
    int counter = 0;
	size_t currentResolution = 0;
	PAL::CameraProperties data;
	bool isDisparityNormalized = true;
    bool flip = false;
	std::vector<PAL::Resolution> resolutions = PAL::GetAvailableResolutions();
 
    PAL::Acknowledgement ack = PAL::LoadProperties("../Explorer/SavedPalProperties.txt", &data);
    if(ack != PAL::SUCCESS)
    {
        printf("Error Loading settings\n");
    }
	while(key != 27)
    {
		PAL::Image left, right, depth, disparity;
		PAL::GrabFrames(&left, &right, &depth, &disparity, isDisparityNormalized, true);
        
        Mat temp, output;
        //Convert PAL::Image to Mat
        l = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
        r = Mat(right.rows, right.cols, CV_8UC3, right.Raw.u8_data);        
        
        
        if(isDisparityNormalized)
        {
			d = Mat(disparity.rows, disparity.cols, CV_8UC1, disparity.Raw.u8_data);
			cvtColor(d, d, CV_GRAY2BGR);
        }
        else 
        {
			d = Mat(disparity.rows, disparity.cols, CV_16SC1, disparity.Raw.u16_data);
			d.convertTo(d, CV_8UC1);

			cvtColor(d, d, CV_GRAY2BGR);
        }
        
        depthMat = Mat(depth.rows, depth.cols, CV_32FC1, depth.Raw.data);
        
        //Vertical concatenation of left and right images into a temp
        vconcat(l, r, temp);
        
        //Vertical concatenation of temp and disparity into the final output
        vconcat(temp, d, output);
                
        //Display the final output image
        imshow( "PAL window", output); 
        
        //imshow( "PAL window", d);  
        
        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
        
        if(key == 'n' || key == 'N')
        {
            isDisparityNormalized = !isDisparityNormalized;
        	if(isDisparityNormalized)
				printf("Displaying normalised disparity\n");
			else	
				printf("Displaying unnormalised disparity\n");	
		}
    	if (key == 'v' || key == 'V')
		{		    
			PAL::CameraProperties prop;
			flip = !flip;
			prop.vertical_flip = flip;
			unsigned int flags = PAL::VERTICAL_FLIP;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key >= '1' && key <= ('0' + resolutions.size()))
		{
			int index = key - '1';
			PAL::CameraProperties prop;
			unsigned int flag = PAL::RESOLUTION;
			prop.resolution = resolutions[index];
			PAL::SetCameraProperties(&prop, &flag);
			printf("Changing the resolution to %dx%d\n",prop.resolution.width,prop.resolution.height);

		}

        
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}
