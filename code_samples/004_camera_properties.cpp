/*

CODE SAMPLE # 004: Changing Camera Properties
This code allows users to modify the camera properties and visualize the changes in the output image. 


>>>>>> Compile this code using the following command....

g++ 004_camera_properties.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DE.so ../lib/libPAL_EDET.so  `pkg-config --libs --cflags opencv`   -O3  -o 004_camera_properties.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl -lX11

>>>>>> Execute the binary file by typing the following command...


004_camera_properties.out


>>>>>> KEYBOARD CONTROLS:

       ESC key closes the window
       Press f/F to toggle filter rgb property
       Press v/V to toggle vertical flip property
       Q & A keys increase and decrease the BRIGHTNESS respectively.
       W & S keys increase and decrease the CONTRAST respectively. 
       E & D keys increase and decrease the SATURATION respectively.
       R & Z keys increase and decrease the GAMMA respectively.
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
#include <X11/Xlib.h>
using namespace cv;
using namespace std;

int main( int argc, char** argv )
{    
    namedWindow( "PAL Camera Properties", WINDOW_NORMAL ); // Create a window for display.
   int width, height;
	if (PAL::Init(width, height, -1) != PAL::SUCCESS) //Connect to the PAL camera
	{
		printf("Init failed\n");
		return 1;
	}
	
    PAL::CameraProperties data; 
    PAL::Acknowledgement ack = PAL::LoadProperties("../Explorer/SavedPalProperties.txt", &data);
    if(ack != PAL::SUCCESS)
    {
        printf("Error Loading settings\n");
    }

	PAL::CameraProperties camera_data;;

	unsigned int flag = PAL::MODE;
	//flag = flag | PAL::FD;
	flag = flag | PAL::NR;
	flag = flag | PAL::FILTER_SPOTS;
	flag = flag | PAL::VERTICAL_FLIP;

	camera_data.mode = PAL::Mode::HIGH_QUALITY_DEPTH;//FAST_DEPTH; // The other available option is PAL::Mode::HIGH_QUALITY_DEPTH
	//prop.fd = 1;
	camera_data.nr = 0;
	camera_data.filter_spots = 1;
	camera_data.vertical_flip =0;
	

	PAL::SetCameraProperties(&camera_data, &flag);

	bool isDisparityNormalized = true;


	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	// Getting Screen resolution 
	Display* disp = XOpenDisplay(NULL);
	Screen*  scrn = DefaultScreenOfDisplay(disp);
	int sc_height = scrn->height;
	int sc_width  = scrn->width;
	
	resizeWindow("PAL Camera Properties", sc_width, sc_height);//width/4, (height/4)*2);
    
    int key = ' ';
    bool filter_spots = true;   
    bool flip = false;
    
    printf("Press ESC to close the window\n");
    printf("Press f/F to toggle filter rgb property\n");
    printf("Press v/V to toggle vertical flip property\n");
    printf("Q & A keys increase and decrease the BRIGHTNESS respectively.\n");
    printf("W & S keys increase and decrease the CONTRAST respectively. \n");
    printf("E & D keys increase and decrease the SATURATION respectively.\n");
    printf("R & Z keys increase and decrease the GAMMA respectively.\n");
    printf("T & G keys increase and decrease the GAIN respectively.\n");
    printf("Y & H keys increase and decrease the WHITE BALANCE TEMPERATURE respectively.\n");
    printf("U & J keys increase and decrease the SHARPNESS respectively.\n");
    printf("I & K keys increase and decrease the EXPOSURE respectively.\n");
    printf("O key toggles AUTO WHITE BALANCE property.\n");
    printf("P key toggles AUTO EXPOSURE property.\n\n");

    printf("C key saves the current left+right panorama image to a numbered file.\n\n");

    printf("N key saves the current camera properties to a file. \n");
    printf("L key loads the camera properties from the saved file.\n");
    
    Mat output = cv::Mat::zeros(height, width, CV_8UC3);
    
    //Display the concatenated image
    imshow( "PAL Camera Properties", output);  
	
	int frame = 0;
	char fileName[1024];
    
    //27 = esc key. Run the loop until the ESC key is pressed
    while(key != 27)
    {
        PAL::Image left, right, depth;
        PAL::GrabFrames(&left, &right, &depth);
        
        //Convert PAL::Image to Mat
        Mat l = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
        Mat r = Mat(right.rows, right.cols, CV_8UC3, right.Raw.u8_data);
	    Mat d = Mat(depth.rows, depth.cols, CV_32FC1, depth.Raw.f32_data);
        cv::Mat tempDisp = cv::Mat::zeros(left.rows, left.cols, CV_8UC1);
		unsigned char *dst = tempDisp.data;
		float *src = (float *)d.data;
		PAL::CameraProperties prop;
		PAL::GetCameraProperties(&prop);
		for (int i = 0; i < left.rows; i++)
		{
			for (int j = 0; j < left.cols; j++)
			{
				float value = *src++;
				value = (value * (1 / 8.0f)) * prop.depth_scale_factor;
				if (value > 255.0f)
					value = 255.0f;
				if (value < 0.0f)
					value = 0.0f;
				*dst++ = (unsigned char)value;
			}
		}

		
		applyColorMap(tempDisp, tempDisp, cv::COLORMAP_JET);
		cvtColor(tempDisp, tempDisp, COLOR_RGB2BGR);
        
        //Vertical concatenation of left and right images
        vconcat(l, tempDisp, output);
        
        //Display the concatenated image
        imshow( "PAL Camera Properties", output);
        
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
            case 'v': // toggle vertical flip
            case 'V':
                flip = !flip;
                camera_data.vertical_flip = flip; 
                flags |= PAL::VERTICAL_FLIP;
            break;
            case 'f': // toggle filter spots
            case 'F':
                filter_spots = !filter_spots;
                camera_data.filter_spots = filter_spots; 
                flags |= PAL::FILTER_SPOTS;
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
            case 'z':  // decrease gamma 
            case 'Z':
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
            printf("vertical_flip      = %d\n", camera_data.vertical_flip);
            printf("filter_spots       = %d\n", camera_data.filter_spots);
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
