/*

CODE SAMPLE # 014: 014_UART_sender



>>>>>> Compile this code using the following command....


g++ 014_UART_sender.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DE.so ../lib/libPAL_EDET.so  ../lib/libPAL_Track.so  `pkg-config --libs --cflags opencv`   -O3  -o 014_UART_sender.out -I../include/ -lv4l2 -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl -lJetsonGPIO -lpthread -w


>>>>>> Execute the binary file by typing the following command...


./014_UART_sender.out


>>>>>> KEYBOARD CONTROLS:

	   ESC key closes the window
	   	Press v/V key to toggle the vertical flip of panorama
		Press f/F to toggle filter rgb property.
		Press d/D to toggle fast depth property
		Press r/R to toggle near range property

*/


# include <stdio.h>
# include <opencv2/opencv.hpp>
# include <chrono>
# include <bits/stdc++.h>
# include "PAL.h"
# include <string.h>
# include <stdlib.h>
# include <stdio.h>

// Linux headers
# include <fcntl.h> // Contains file controls like O_RDWR
# include <errno.h> // Error integer and strerror() function
# include <termios.h> // Contains POSIX terminal control definitions
# include <unistd.h> // write(), read(), close()



static bool g_bExit = false;

void signalHandler( int signum )
{
	g_bExit = true;
}

namespace PAL
{
	namespace Internal
	{
		void EnableDepth(bool flag);
		void MinimiseCompute(bool flag);
	}
}


using namespace cv;
using namespace std;

using namespace std::chrono;

namespace PAL
{

	int RunTrack(cv::Mat& img, cv::Mat& depth, vector<vector<float>> &boxes, 
	    vector<int> &ids, vector<float> &depthValues, vector<Scalar> &colours);
}

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




int main(int argc, char *argv[])
{
   	
    signal(SIGINT, signalHandler);

    bool useDepth = false;

    if(argc>1)
    {
        int input = atoi(argv[1]);
        useDepth = (input>0) ? true: false;
    }    

    PAL::Internal::EnableDepth(useDepth);
    PAL::Internal::MinimiseCompute(!useDepth);

    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    int serial_port = open("/dev/ttyS0", O_RDWR);

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

	PAL::CameraProperties prop;

	unsigned int flag = PAL::MODE;
	flag = flag | PAL::FD;
	flag = flag | PAL::NR;
	flag = flag | PAL::FILTER_SPOTS;
	flag = flag | PAL::VERTICAL_FLIP;

	prop.mode = PAL::Mode::TRACKING;
	prop.fd = 1;
	prop.nr = 0;
	prop.filter_spots = 1;
	prop.vertical_flip = 0;
	PAL::SetCameraProperties(&prop, &flag);

	printf("Press Ctrl+C to exit the app\n");   

	vector<vector<float>> boxes; 
	vector<int> ids; 
	vector<float> depthValues; 
	vector<Scalar> colours; 
	int num;	

    const char* Start = "S"; // Mark start of the frame
    const char* End = "E";   // Mark end of the frame
    const char* new_frame = "D"; // Start of new index 
    const char* Check = "C"; // Mark Checksum 


	//27 = esc key. Run the loop until the ESC key is pressed
	while(!g_bExit)
	{
		PAL::Image left, right, depth, disparity;
		Mat img, d;
		if (useDepth)
			PAL::GrabFrames(&left, &right, &depth);
		else
			PAL::GrabFrames(&left, &right);
		
		//Convert PAL::Image to Mat
		img = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
		if (useDepth)
		{
			d = Mat(depth.rows, depth.cols, CV_32FC1, depth.Raw.f32_data);
		}
		else
		{
			d = cv::Mat::zeros(cv::Size(1, 1), CV_32FC1);
		}
		
		num = PAL::RunTrack(img, d, boxes, ids, depthValues, colours);

		if(num)
		{
            // Writing Start of the frame to the serial port
            write(serial_port, Start, 1);    

            printf("[INFO] START OF NEW FRAME\n");
            char num_buffer[int_length(num)]; 
            my_itoa(num, num_buffer);
            write(serial_port, num_buffer, sizeof(num_buffer));
            printf("Num of person detected: %d\n" , num);
		        
            if(useDepth)
            {
                for(int i=0; i<num; i++)
                {
                    bool invalid = std::isnan(depthValues[i]) || (depthValues[i]<0) || (depthValues[i]>1000);                    
                    depthValues[i] = (invalid) ? 100: depthValues[i];
                }

                for(int i=0; i<num; i++)
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
		
		boxes.clear();
		ids.clear();
		if(useDepth)
		        depthValues.clear();
		colours.clear();		
		
	}

	printf("exiting the application\n");
	PAL::Destroy();

	return 0;
}
