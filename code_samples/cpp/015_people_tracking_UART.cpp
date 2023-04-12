/*

CODE SAMPLE # 015: People Tracking
This code will grab the detection and depth data and transmit the same over uart.


>>>>>> Compile this code using the following command....

./compile.sh 015_people_tracking_UART.cpp


>>>>>> Execute the binary file by typing the following command...

./015_people_tracking_UART.out


>>>>>> KEYBOARD CONTROLS:

    Press CTRL+C to close the application.
      
*/


# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"
#include "TimeLogger.h"
#include <time.h>
#include <unistd.h>

#include<sys/time.h>
#include <chrono>
#include <iomanip>
// Linux headers
# include <fcntl.h> // Contains file controls like O_RDWR
# include <errno.h> // Error integer and strerror() function
# include <termios.h> // Contains POSIX terminal control definitions
# include <unistd.h> // write(), read(), close()
#include <bits/stdc++.h>


static bool g_bExit = false;
int person_detected = 0;
std::vector<int> depthValues;
void signalHandler( int signum )
{
	g_bExit = true;
}

using namespace cv;
using namespace std;

char *my_itoa(int num, char *str)
{
        if(str == NULL)
        {
                return NULL;
        }
        sprintf(str, "%d", num);
        return str;
}

//calculates the number of digits of an integer
int int_length(__uint16_t a)
{  
    int b=1;            
    while(a/=10)        
        b++;
    return b;
}


std::vector<int> depth_vec;

Scalar get_color(int idx)
{
    idx += 3;
    return Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}

std::string precision_string(float num, int precision=1)
{
    std::string num_string = std::to_string(num);
    return num_string.substr(0, num_string.find(".")+1+precision);
}

void drawOnImage(cv::Mat &img, const PAL::Data::TrackingResults &data, int mode,
    bool ENABLEDEPTH=false, bool ENABLE3D=false)
{
    vector<std::string> classes = {"person","bicycle","car","motorcycle","airplane",
    "bus","train","truck","boat","traffic light","fire hydrant",
    "stop sign","parking meter","bench","bird","cat","dog","horse",
    "sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat",
    "baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot",
    "hot dog","pizza","donut","cake","chair","couch","potted plant",
    "bed","dining table","toilet","tv","laptop","mouse","remote",
    "keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"};

    bool only_detection = (mode == PAL::Tracking_Mode::OBJECT_DETECTION) ? true : false;
    if(!ENABLEDEPTH)
        ENABLE3D = false;

    int no_of_persons = data.trackingData[PAL::States::OK].size();
    person_detected = no_of_persons;
    depth_vec.resize(no_of_persons);
    depthValues.resize(person_detected);
    
    for (int i = 0; i < no_of_persons; i++)
    {
        cv::Scalar colors = get_color((int)data.trackingData[PAL::States::OK][i].t_track_id);

        int x1,y1,x2,y2;
        x1 = (int)data.trackingData[PAL::States::OK][i].boxes.x1;
        y1 = (int)data.trackingData[PAL::States::OK][i].boxes.y1;
        x2 = (int)data.trackingData[PAL::States::OK][i].boxes.x2;
        y2 = (int)data.trackingData[PAL::States::OK][i].boxes.y2;

        float x3D, y3D, z3D, depth_value;
        x3D = data.trackingData[PAL::States::OK][i].locations_3d.x;
        y3D = data.trackingData[PAL::States::OK][i].locations_3d.y;
        z3D = data.trackingData[PAL::States::OK][i].locations_3d.z;

        depth_value = sqrt(x3D*x3D + y3D*y3D);
		depthValues[i] = depth_value*100;
        int fontface = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 0.4;
        int thickness = 1;
        int baseline = 0;
        int height_mul = 3;

        std::string label1, label2;
        cv::Size text1, text2;
        
        if(only_detection)
        {
            label1 = "Class= " + classes[ round(data.trackingData[PAL::States::OK][i].t_label) ];
        }
        else
        {
            label1 = "ID=" + to_string((int)data.trackingData[PAL::States::OK][i].t_track_id) + 
                ", "+classes[ round(data.trackingData[PAL::States::OK][i].t_label) ];
        }

        if(ENABLEDEPTH)
        {
            if(ENABLE3D)
            {
                label1 += ", Depth=" + precision_string(depth_value,1) + "m";
                label2 = "x:" + precision_string(x3D,1) + "m, y:" + precision_string(y3D,1) + 
                    "m, z:" + precision_string(z3D,1) + "m";
            }
            else
            {
                label2 = "Depth=" + precision_string(depth_value,1) + "m";
            
            }
            text2 = cv::getTextSize(label2, fontface, scale, thickness, &baseline); 
            baseline += thickness;
        }
        
        text1 = cv::getTextSize(label1, fontface, scale, thickness, &baseline); 
        baseline += thickness;

        int box_height = text1.height + text2.height + baseline + 3;
        int box_width = std::max(text1.width, text2.width) + 2;

        cv::rectangle(img, Rect(x1, y1-box_height-1, box_width, box_height), Scalar(255,255,255), cv::FILLED);        

        cv::putText(img, label1, Point(x1, (int)(y1-text2.height-baseline-2)), fontface, scale, Scalar(0, 0, 255), thickness, cv::LINE_AA);
        if(ENABLEDEPTH)
        {
            cv::putText(img, label2, Point(x1, (int)(y1-baseline/2 -1)), fontface, scale, Scalar(0, 0, 255), thickness, cv::LINE_AA); 
        }

        cv::rectangle(img, Rect(x1, y1, x2, y2), colors, 2);
    }
    
    putText(img, format("num: %d", no_of_persons), Point(0, 30), 0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
}


int main( int argc, char** argv )
{
	signal(SIGINT, signalHandler);
	bool useDepth = false;
	bool enableDepth = false;
 	if(argc>1)
    {
        int input = atoi(argv[1]);
        useDepth = (input>0) ? true: false;
        if(useDepth)
        {
        	enableDepth = true;
        	SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_ON);
    	}
    	else
    	{
    		enableDepth = false;
    		PAL::SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_OFF);
    	}
    } 
    namedWindow( "PAL OBJECT_TRACKING_UART", WINDOW_NORMAL ); // Create a window for display.

	// Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    int serial_port = open("/dev/ttyTHS0", O_RDWR);

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) 
    {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return 1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
    {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return 1;
    }



    int width, height;
    std::vector<int> camera_indexes{5};
    PAL::Mode def_mode = PAL::Mode::LASER_SCAN;

    //Start the PAL application
    if (PAL::Init(width, height, camera_indexes, &def_mode) != PAL::SUCCESS) //Connect to the PAL camera
    {
        cout<<"Init failed"<<endl;
        return 1;
    }

    //Select which mode you want to run the application in.
    PAL::SetAPIMode(PAL::API_Mode::TRACKING);
    usleep(1000000);

    PAL::CameraProperties data;
    PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &data);

    if(ack_load != PAL::SUCCESS)
    {
        cout<<"Error Loading settings! Loading default values."<<endl;
    }

    bool filter_spots = true;
    bool flip = true;
    bool fd = true;
    
    bool enable3Dlocation = false;
    

    int tracking_mode = PAL::Tracking_Mode::PEOPLE_TRACKING;
    int success = PAL::SetModeInTracking(tracking_mode);

    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed at otheir original resolution.
    resizeWindow("PAL OBJECT_TRACKING_UART", width, height);

    int key = ' ';

    
    
    const char* Start = "S"; // Mark start of the frame
    const char* End = "E";   // Mark end of the frame
    const char* new_frame = "D"; // Start of new index 
    const char* Check = "C"; // Mark Checksum 
    
	extern bool camera_changed;
	
	std::cout << "Press CTRL+C to close the application." << std::endl;

	//27 = esc key. Run the loop until the ESC key is pressed
	while(!g_bExit)
	{
		
		if(camera_changed)
		{
			break;
		}

        std::vector<PAL::Data::TrackingResults> data;
        data =  PAL::GrabTrackingData();    

        cv::Mat display = data[0].left;
        drawOnImage(display, data[0], tracking_mode, enableDepth, enable3Dlocation);
        if(person_detected)
		{
		    // Writing Start of the frame to the serial port
		    write(serial_port, Start, 1);    

		    printf("[INFO] START OF NEW FRAME\n");
		    char num_buffer[int_length(person_detected)]; 
		    my_itoa(person_detected, num_buffer);
		    write(serial_port, num_buffer, sizeof(num_buffer));
		    printf("Num of person detected: %d\n" , person_detected);
				
		    if(useDepth)
		    {
		        for(int i=0; i<person_detected; i++)
		        {
		            bool invalid = std::isnan(depthValues[i]) || (depthValues[i]<0) || (depthValues[i]>1000);                    
		            depthValues[i] = (invalid) ? 100: depthValues[i];
		        }

		        for(int i=0; i<person_detected; i++)
		        {
		            char buffer[int_length(depthValues[i])]; 
		            my_itoa(depthValues[i], buffer);
		            printf(" DISTANCE : %d cm\n", (int)depthValues[i]);

		            write(serial_port, new_frame, 1);
		            write(serial_port, buffer, sizeof(buffer));
		        }
		    }
		    // Writing End of the frame to the serial port		                
		    write(serial_port, End, 1);			                
		    printf("[INFO] END OF THE FRAME\n\n");
		}
		
		
    }

    printf("exiting the application\n");
    PAL::Destroy();
   
    return 0;
}

