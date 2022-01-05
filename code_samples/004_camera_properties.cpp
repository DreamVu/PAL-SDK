/*

CODE SAMPLE # 004: Changing Camera Properties
This code allows users to modify the camera properties and visualize the changes in the output image. 


>>>>>> Compile this code using the following command....


g++ 004_camera_properties.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -g  -o 004_camera_properties.out -I../include/ -lv4l2 -lpthread


>>>>>> Execute the binary file by typing the following command...


004_camera_properties.out


>>>>>> KEYBOARD CONTROLS:

       ESC key closes the window
       Q & A keys increase and decrease the BRIGHTNESS respectively.
       W & S keys increase and decrease the CONTRAST respectively. 
       E & D keys increase and decrease the SATURATION respectively.
       R & F keys increase and decrease the GAMMA respectively.
       T & G keys increase and decrease the GAIN respectively.
       Y & H keys increase and decrease the WHITE BALANCE TEMPERATURE respectively.
       U & J keys increase and decrease the SHARPNESS respectively.
       I & K keys increase and decrease the EXPOSURE respectively.
       O key toggles AUTO WHITE BALANCE property.
       P key toggles AUTO EXPOSURE property.

       C key saves the current left+right panorama image to a numbered file.

       N key saves the current camera properties to a file. 
       L key loads the camera properties from the saved file.
*/

# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"

using namespace cv;
using namespace std;



int main( int argc, char** argv )
{

    int camera_index=1;
    cout << "Please enter you camera index(default is 1) and press Enter: ";
    cin >> camera_index;
    PAL::CameraProperties camera_data;
    
    namedWindow( "PAL window", WINDOW_NORMAL ); // Create a window for display.
    
    int width, height;
    if(PAL::Init(width, height,camera_index) != PAL::SUCCESS) //Connect to the PAL camera
    {
        printf("Init failed\n");
        return 1;
    }
    
    PAL::GetCameraProperties(&camera_data);
    
    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed after scaling by scaleFactor w.r.t their original resolution.
    //Since the panoramas are vertically stacked, the window height should be twice the scaled height
    float scaleFactor = 5.0f/8.0f; 
    resizeWindow("PAL window", (int)(width*scaleFactor), int(height*2*scaleFactor));
    
    int key = ' ';
    
    printf("Press ESC to close the window\n");

	printf("The image resolution is .... %dx%d\n", width, height);


    
    Mat output = cv::Mat::zeros(height, width, CV_8UC3);
    
    //Display the concatenated image
    imshow( "PAL window", output);  
	
	int frame = 0;
	char fileName[1024];
    
    //27 = esc key. Run the loop until the ESC key is pressed
    while(key != 27)
    {
        PAL::Image left, right;
        PAL::GrabFrames(&left, &right);
        
        //Convert PAL::Image to Mat
        Mat l = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
        Mat r = Mat(right.rows, right.cols, CV_8UC3, right.Raw.u8_data);
        
        //Vertical concatenation of left and right images
        vconcat(l, r, output);
        
        //Display the concatenated image
        imshow( "PAL window", output);
        
        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
        
        unsigned int flags = 0;

        switch (key)
        {   
            case ' ':   
                camera_data.brightness = PAL::CameraProperties::DEFAULT_BRIGHTNESS;
                camera_data.contrast = PAL::CameraProperties::DEFAULT_CONTRAST;     
                camera_data.saturation = PAL::CameraProperties::DEFAULT_SATURATION;
                camera_data.gamma = PAL::CameraProperties::DEFAULT_GAMMA;
                camera_data.gain = PAL::CameraProperties::DEFAULT_GAIN;
                camera_data.white_bal_temp = PAL::CameraProperties::DEFAULT_WHITE_BAL_TEMP;
                camera_data.sharpness = PAL::CameraProperties::DEFAULT_SHARPNESS; 
                camera_data.exposure = PAL::CameraProperties::DEFAULT_EXPOSURE;  
                camera_data.auto_white_bal = PAL::CameraProperties::DEFAULT_AUTO_WHITE_BAL;
                camera_data.auto_exposure = PAL::CameraProperties::DEFAULT_AUTO_EXPOSURE;
                flags = PAL::ALL; 
            break;


            case 'q':  // increase brightness 
            case 'Q':
                camera_data.brightness += 1;
                if(camera_data.brightness > PAL::CameraProperties::MAX_BRIGHTNESS) camera_data.brightness = PAL::CameraProperties::MAX_BRIGHTNESS; 
                flags |= PAL::BRIGHTNESS;
            break;

            case 'a':  // decrease brightness
            case 'A':
                camera_data.brightness -= 1;
                if(camera_data.brightness < PAL::CameraProperties::MIN_BRIGHTNESS) camera_data.brightness = PAL::CameraProperties::MIN_BRIGHTNESS;
                flags |= PAL::BRIGHTNESS;
            break;

            case 'w':  // increase contrast 
            case 'W':
                camera_data.contrast += 1;
                if(camera_data.contrast > PAL::CameraProperties::MAX_CONTRAST) camera_data.contrast = PAL::CameraProperties::MAX_CONTRAST; 
                flags |= PAL::CONTRAST;
            break;

            case 's':  // decrease contrast
            case 'S':
                camera_data.contrast -= 1;
                if(camera_data.contrast < PAL::CameraProperties::MIN_CONTRAST) camera_data.contrast = PAL::CameraProperties::MIN_CONTRAST;
                flags |= PAL::CONTRAST;
            break;

            case 'e':  // increase saturation 
            case 'E':
                camera_data.saturation += 1;
                if(camera_data.saturation > PAL::CameraProperties::MAX_SATURATION) camera_data.saturation = PAL::CameraProperties::MAX_SATURATION; 
                flags |= PAL::SATURATION;
            break;

            case 'd':  // decrease saturation
            case 'D':
                camera_data.saturation -= 1;
                if(camera_data.saturation < PAL::CameraProperties::MIN_SATURATION) camera_data.saturation = PAL::CameraProperties::MIN_SATURATION;
                flags |= PAL::SATURATION;
            break;


            case 'r':  // increase gamma
            case 'R':
                camera_data.gamma += 10;
                if(camera_data.gamma > PAL::CameraProperties::MAX_GAMMA) camera_data.gamma = PAL::CameraProperties::MAX_GAMMA; 
                flags |= PAL::GAMMA;
            break;

            case 'f':  // decrease gamma 
            case 'F':
                camera_data.gamma -= 10;
                if(camera_data.gamma < PAL::CameraProperties::MIN_GAMMA) camera_data.gamma = PAL::CameraProperties::MIN_GAMMA;
                flags |= PAL::GAMMA;
            break;

            case 't':  // increase gain 
            case 'T':
                camera_data.gain += 1;
                if(camera_data.gain > PAL::CameraProperties::MAX_GAIN) camera_data.gain = PAL::CameraProperties::MAX_GAIN;
                flags |= PAL::GAIN;
            break;

            case 'g':  // decrease gain 
            case 'G':
                camera_data.gain -= 1;
                if(camera_data.gain < PAL::CameraProperties::MIN_GAIN) camera_data.gain = PAL::CameraProperties::MIN_GAIN;
                flags |= PAL::GAIN;
            break;

            case 'y':  // increase white balance temperature
            case 'Y':
                camera_data.white_bal_temp += 200;
                if(camera_data.white_bal_temp > PAL::CameraProperties::MAX_WHITE_BAL_TEMP) camera_data.white_bal_temp = PAL::CameraProperties::MAX_WHITE_BAL_TEMP;
                flags |= PAL::WHITE_BAL_TEMP;
            break;

            case 'h':  // decrease white balance temperature
            case 'H':
                camera_data.white_bal_temp -= 200;
                if(camera_data.white_bal_temp < PAL::CameraProperties::MIN_WHITE_BAL_TEMP) camera_data.white_bal_temp = PAL::CameraProperties::MIN_WHITE_BAL_TEMP;
                flags |= PAL::WHITE_BAL_TEMP;
            break;
            
            case 'u':  // increase sharpness 
            case 'U':
                camera_data.sharpness += 1;
                if(camera_data.sharpness > PAL::CameraProperties::MAX_SHARPNESS) camera_data.sharpness = PAL::CameraProperties::MAX_SHARPNESS;
                flags |= PAL::SHARPNESS;
            break;

            case 'j':  // decrease sharpness 
            case 'J':
                camera_data.sharpness -= 1;
                if(camera_data.sharpness < PAL::CameraProperties::MIN_SHARPNESS) camera_data.sharpness = PAL::CameraProperties::MIN_SHARPNESS;
                flags |= PAL::SHARPNESS;
            break;


            case 'i':  // increase exposure 
            case 'I':
                camera_data.exposure += 50;
                if(camera_data.exposure > PAL::CameraProperties::MAX_EXPOSURE) camera_data.exposure = PAL::CameraProperties::MAX_EXPOSURE;
                flags |= PAL::EXPOSURE;
            break;

            case 'k':  // decrease exposure 
            case 'K':
                camera_data.exposure -= 50;
                if(camera_data.exposure < PAL::CameraProperties::MIN_EXPOSURE) camera_data.exposure = PAL::CameraProperties::MIN_EXPOSURE;
                flags |= PAL::EXPOSURE;
            break;


            case 'o':  // Toggle auto white balance temperature
            case 'O':
                camera_data.auto_white_bal = !camera_data.auto_white_bal;
                flags |= PAL::AUTO_WHITE_BAL;
                break;
                  
            case 'p':  // Toggle auto exposure 
            case 'P':
                camera_data.auto_exposure = !camera_data.auto_exposure;
                flags |= PAL::AUTO_EXPOSURE;
                break;

            case 'c': // Saves current left+right image to disk as a .png file 
            case 'C': 
                sprintf(fileName,"./pal_image_%03d.png", frame++);
                cv::imwrite(fileName, output);
                break;
                
            case 'n': // Saves camera properties to file
            case 'N':
                PAL::SaveProperties("properties.txt");
                printf(">>>>>> SAVED THE PROPERTIES >>>>\n");
                break;
                
            case 'l': // Loads camera properties from file 
            case 'L':
                PAL::LoadProperties("properties.txt", &camera_data);
                break;
                
        }
        
        if(flags != 0)
        {
         PAL::SetCameraProperties(&camera_data, &flags);
         printf("Camera Properties....\n");
         printf("brightness         = %d\n", camera_data.brightness);
         printf("contrast           = %d\n", camera_data.contrast);
         printf("saturation         = %d\n", camera_data.saturation);
         printf("gamma              = %d\n", camera_data.gamma);
         printf("gain               = %d\n", camera_data.gain);
         printf("white_bal_temp     = %d\n", camera_data.white_bal_temp);
         printf("sharpness          = %d\n", camera_data.sharpness);
         printf("exposure           = %d\n", camera_data.exposure);
         printf("auto_white_bal     = %d\n", camera_data.auto_white_bal);
         printf("auto_exposure      = %d\n\n", camera_data.auto_exposure);
        }
        
    }

    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}