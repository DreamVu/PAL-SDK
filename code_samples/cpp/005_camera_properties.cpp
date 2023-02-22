/*

CODE SAMPLE # 005: PAL Camera Properties
This code will grab the left & depth panorama and display in a window using opencv


>>>>>> Compile this code using the following command....

./compile.sh 005_camera_properties.cpp

>>>>>> Execute the binary file by typing the following command...

./005_camera_properties.out


>>>>>> KEYBOARD CONTROLS:

ESC key closes the window
Press Q & A keys increase and decrease the BRIGHTNESS respectively.
Press W & S keys increase and decrease the CONTRAST respectively.
Press E & D keys increase and decrease the SATURATION respectively.
Press R & Z keys increase and decrease the GAMMA respectively.
Press T & G keys increase and decrease the GAIN respectively.
Press Y & H keys increase and decrease the WHITE BALANCE TEMPERATURE respectively.
Press U & J keys increase and decrease the SHARPNESS respectively.
Press I & K keys increase and decrease the EXPOSURE respectively.
Press O key toggles AUTO WHITE BALANCE property.
Press P key toggles AUTO EXPOSURE property.
Press C key saves the current left+depth panorama image to a numbered file.
Press N key saves the current camera properties to a file.
Press L key loads the camera properties from the saved file.
Press V key toggles VERTICAL FLIP property.
Press F key toggles FILTER SPOTS property.  

*/


# include <stdio.h>
# include <opencv2/opencv.hpp>
# include "PAL.h"
#include "TimeLogger.h"
#include <time.h>
#include <unistd.h>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
	// Create a window for display.
	namedWindow( "PAL Camera Properties", WINDOW_NORMAL ); 

	int width, height;
	PAL::Mode mode = PAL::Mode::LASER_SCAN;

	std::vector<int> camera_indexes{5};
	
	if(argc > 1) 
		camera_indexes[0] = std::atoi(argv[1]);


	PAL::Mode def_mode = PAL::Mode::LASER_SCAN;

	char path[1024];
	sprintf(path,"/usr/local/bin/data/pal/data%d/",camera_indexes[0]);

	char path2[1024];
	sprintf(path2,"/usr/local/bin/data/pal/data%d/",6);

	PAL::SetPathtoData(path, path2);

	//Connect to the PAL camera
	if (PAL::Init(width, height, camera_indexes, &def_mode) != PAL::SUCCESS) 
	{
		cout<<"Init failed"<<endl;
		return 1;
	}

	PAL::SetAPIMode(PAL::API_Mode::DEPTH);
	usleep(1000000);

	PAL::CameraProperties camera_data;
	PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &camera_data);

	if(ack_load != PAL::SUCCESS)
	{
		cout<<"Error Loading settings! Loading default values."<<endl;
	}

	//discarding initial frames
	std::vector<PAL::Data::ODOA_Data> discard;
	for(int i=0; i<5;i++)
		discard =  PAL::GrabRangeScanData();		

	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at their original resolution.
	resizeWindow("PAL Camera Properties", width, height);

	int key = ' ';

	cout<<"Press ESC to close the window."<<endl;
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
	printf("C key saves the current left+depth panorama image to a numbered file.\n\n");
	printf("N key saves the current camera properties to a file. \n");
    printf("L key loads the camera properties from the saved file.\n");
    printf("V key toggles VERTICAL FLIP property. \n");
    printf("F key toggles FILTER SPOTS property. \n");
    
    
	Mat output = cv::Mat::zeros(height, width, CV_8UC3);

	//Display the overlayed image
	imshow( "PAL Camera Properties", output);

	int frame = 0;
	char fileName[1024];

	//27 = esc key. Run the loop until the ESC key is pressed
	while(key != 27)
	{

		std::vector<PAL::Data::ODOA_Data> data;

		data =  PAL::GrabRangeScanData();	
		
		Mat display;
		Mat l = data[0].left;
		Mat d = data[0].distance.clone();
		d.convertTo(d, CV_8UC1);
		cvtColor(d, d, cv::COLOR_GRAY2BGR);

		//Vertical concatenation of rgb and depth into the final output
		vconcat(l, d, display);

		//Display the depth with rgb panorama
		imshow( "PAL Camera Properties", display);  

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;
		
		unsigned long int flags = 0;

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
            // increase brightness
            case 'q':   
            case 'Q':
                camera_data.brightness += 1;
                if(camera_data.brightness > PAL::CameraProperties::MAX_BRIGHTNESS) camera_data.brightness = PAL::CameraProperties::MAX_BRIGHTNESS; 
                flags |= PAL::BRIGHTNESS;
            break;
            // decrease brightness
            case 'a':  
            case 'A':
                camera_data.brightness -= 1;
                if(camera_data.brightness < PAL::CameraProperties::MIN_BRIGHTNESS) camera_data.brightness = PAL::CameraProperties::MIN_BRIGHTNESS;
                flags |= PAL::BRIGHTNESS;
            break;
            // increase contrast 
            case 'w':  
            case 'W':
                camera_data.contrast += 1;
                if(camera_data.contrast > PAL::CameraProperties::MAX_CONTRAST) camera_data.contrast = PAL::CameraProperties::MAX_CONTRAST; 
                flags |= PAL::CONTRAST;
            break;
            // decrease contrast
            case 's':  
            case 'S':
                camera_data.contrast -= 1;
                if(camera_data.contrast < PAL::CameraProperties::MIN_CONTRAST) camera_data.contrast = PAL::CameraProperties::MIN_CONTRAST;
                flags |= PAL::CONTRAST;
            break;
            // increase saturation
            case 'e':   
            case 'E':
                camera_data.saturation += 1;
                if(camera_data.saturation > PAL::CameraProperties::MAX_SATURATION) camera_data.saturation = PAL::CameraProperties::MAX_SATURATION; 
                flags |= PAL::SATURATION;
            break;
            // decrease saturation
            case 'd':  
            case 'D':
                camera_data.saturation -= 1;
                if(camera_data.saturation < PAL::CameraProperties::MIN_SATURATION) camera_data.saturation = PAL::CameraProperties::MIN_SATURATION;
                flags |= PAL::SATURATION;
            break;
             // increase gamma
            case 'r': 
            case 'R':
                camera_data.gamma += 10;
                if(camera_data.gamma > PAL::CameraProperties::MAX_GAMMA) camera_data.gamma = PAL::CameraProperties::MAX_GAMMA; 
                flags |= PAL::GAMMA;
            break;
            // decrease gamma
            case 'z':   
            case 'Z':
                camera_data.gamma -= 10;
                if(camera_data.gamma < PAL::CameraProperties::MIN_GAMMA) camera_data.gamma = PAL::CameraProperties::MIN_GAMMA;
                flags |= PAL::GAMMA;
            break;
            // increase gain
            case 't':   
            case 'T':
                camera_data.gain += 1;
                if(camera_data.gain > PAL::CameraProperties::MAX_GAIN) camera_data.gain = PAL::CameraProperties::MAX_GAIN;
                flags |= PAL::GAIN;
            break;
            // decrease gain
            case 'g':   
            case 'G':
                camera_data.gain -= 1;
                if(camera_data.gain < PAL::CameraProperties::MIN_GAIN) camera_data.gain = PAL::CameraProperties::MIN_GAIN;
                flags |= PAL::GAIN;
            break;
            // increase white balance temperature
            case 'y':  
            case 'Y':
                camera_data.white_bal_temp += 200;
                if(camera_data.white_bal_temp > PAL::CameraProperties::MAX_WHITE_BAL_TEMP) camera_data.white_bal_temp = PAL::CameraProperties::MAX_WHITE_BAL_TEMP;
                flags |= PAL::WHITE_BAL_TEMP;
            break;
            // decrease white balance temperature
            case 'h':  
            case 'H':
                camera_data.white_bal_temp -= 200;
                if(camera_data.white_bal_temp < PAL::CameraProperties::MIN_WHITE_BAL_TEMP) camera_data.white_bal_temp = PAL::CameraProperties::MIN_WHITE_BAL_TEMP;
                flags |= PAL::WHITE_BAL_TEMP;
            break;
            // increase sharpness 
            case 'u':   
            case 'U':
                camera_data.sharpness += 1;
                if(camera_data.sharpness > PAL::CameraProperties::MAX_SHARPNESS) camera_data.sharpness = PAL::CameraProperties::MAX_SHARPNESS;
                flags |= PAL::SHARPNESS;
            break;
            // decrease sharpness
            case 'j':   
            case 'J':
                camera_data.sharpness -= 1;
                if(camera_data.sharpness < PAL::CameraProperties::MIN_SHARPNESS) camera_data.sharpness = PAL::CameraProperties::MIN_SHARPNESS;
                flags |= PAL::SHARPNESS;
            break;
            // increase exposure
            case 'i':   
            case 'I':
                camera_data.exposure += 50;
                if(camera_data.exposure > PAL::CameraProperties::MAX_EXPOSURE) camera_data.exposure = PAL::CameraProperties::MAX_EXPOSURE;
                flags |= PAL::EXPOSURE;
            break;
            // decrease exposure 
            case 'k':  
            case 'K':
                camera_data.exposure -= 50;
                if(camera_data.exposure < PAL::CameraProperties::MIN_EXPOSURE) camera_data.exposure = PAL::CameraProperties::MIN_EXPOSURE;
                flags |= PAL::EXPOSURE;
            break;
            // Toggle auto white balance temperature
            case 'o':  
            case 'O':
                camera_data.auto_white_bal = !camera_data.auto_white_bal;
                flags |= PAL::AUTO_WHITE_BAL;
            break;  
            // Toggle auto exposure                   
            case 'p':   
            case 'P':
                camera_data.auto_exposure = !camera_data.auto_exposure;
                flags |= PAL::AUTO_EXPOSURE;
            break;
            // Saves current left+depth image to disk as a .png file    
            case 'c':  
            case 'C': 
                sprintf(fileName,"./pal_image_%03d.png", frame++);
                cv::imwrite(fileName, display);
            break;
            // Saves camera properties to file           
            case 'n': 
            case 'N':
                PAL::SaveProperties("properties.txt");
                printf(">>>>>> SAVED THE PROPERTIES >>>>\n");
		    break;
		    // Loads camera properties from file
            case 'l':  
            case 'L':
                PAL::LoadProperties("properties.txt", &camera_data);
                break;
            //Toogle vertical flip     
            case 'V':  
            case 'v':
                camera_data.vertical_flip = !camera_data.vertical_flip;
                flags |= PAL::VERTICAL_FLIP;
            break; 
            // Toggle Filter spots
            case 'F':  
            case 'f':
                camera_data.filter_spots = !camera_data.filter_spots;
                flags |= PAL::FILTER_SPOTS;
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

