/*

CODE SAMPLE # 015: People Tracking
This code will grab the detection and depth data and transmit the same over uart.

>>>>>> Compile this code using the following command....

./compile.sh 015_people_tracking_UART.cpp

>>>>>> Execute the binary file by typing the following command...

./015_people_tracking_UART.out
      
*/

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "PAL.h"
#include <iomanip>
#include <JetsonGPIO.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <csignal>
#include <string>

using namespace cv;
using namespace std;

const string PORT_NVIDIA_NX = "/dev/ttyTHS0";
const string PORT_DREAMVU_NX = "/dev/ttyTCU0";
const string PORT_NVIDIA_NANO = "/dev/ttyS0";
const string PORT_DREAMVU_NANO = "/dev/ttyS0";

string getPortName()
{
    int board_type, board_model;
    PAL::GetBoardTypeAndModel(board_type, board_model);

    string port_name;
    if(board_model == 0) // NX
    {
        if(board_type == 0) // NVIDIA BOARD
        {
            port_name = PORT_NVIDIA_NX;
        }
        else if(board_type == 1) // DREAMVU BOARD
        {
            port_name = PORT_DREAMVU_NX;
        }
    }
    else if(board_model == 1) // NANO
    {
        if(board_type == 0) // NVIDIA BOARD
        {
            port_name = PORT_NVIDIA_NANO;
        }
        else if(board_type == 1) // DREAMVU BOARD
        {
            port_name = PORT_DREAMVU_NANO;
        }
    }
    else
    {
        cerr << "Device is neither NX nor NANO." << endl; 
        cerr << "To get the port name for your device, select a port listed under /dev/tty* and use the application 'putty' to check if connection is being established" << endl;
        exit(1);
    }

    return port_name;
}

static bool g_bExit = false;
void signalHandler( int signum )
{
    g_bExit = true;
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

std::vector<int> depthValues;

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

void drawOnImage(cv::Mat &img, const PAL::Data::Tracking_Data &data, int mode,
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

    int no_of_persons = data.tracking_info[PAL::States::OK].size();
    depthValues.resize(no_of_persons);
    
    for (int i = 0; i < no_of_persons; i++)
    {
        cv::Scalar colors = get_color((int)data.tracking_info[PAL::States::OK][i].t_track_id);

        int x1,y1,x2,y2;
        x1 = (int)data.tracking_info[PAL::States::OK][i].boxes.x1;
        y1 = (int)data.tracking_info[PAL::States::OK][i].boxes.y1;
        x2 = (int)data.tracking_info[PAL::States::OK][i].boxes.x2;
        y2 = (int)data.tracking_info[PAL::States::OK][i].boxes.y2;

        float x3D, y3D, z3D, depth_value;
        x3D = data.tracking_info[PAL::States::OK][i].locations_3d.x;
        y3D = data.tracking_info[PAL::States::OK][i].locations_3d.y;
        z3D = data.tracking_info[PAL::States::OK][i].locations_3d.z;

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
            label1 = "Class= " + classes[ round(data.tracking_info[PAL::States::OK][i].t_label) ];
        }
        else
        {
            label1 = "ID=" + to_string((int)data.tracking_info[PAL::States::OK][i].t_track_id) + 
                ", "+classes[ round(data.tracking_info[PAL::States::OK][i].t_label) ];
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

        int text_bg_x1 = x1 < 0 ? 0 : x1;
        int text_bg_y1 = y1-box_height-1 < 0 ? 0 : y1-box_height-1;

        if(text_bg_x1 + box_width > img.cols)
        {
            text_bg_x1 = img.cols - (box_width);
        }

        cv::rectangle(img, cv::Rect(text_bg_x1, text_bg_y1, box_width, box_height), cv::Scalar(255,255,255), cv::FILLED); 

        int text_line1_x1 = text_bg_x1;
        int text_line1_y1 = y1-box_height-1 < 0 ? (int)(text1.height + baseline/2) : (int)y1-text2.height-baseline-2;

        cv::putText(img, label1, cv::Point(text_line1_x1, text_line1_y1), fontface, scale, cv::Scalar(0, 0, 255), thickness, cv::LINE_AA);
        if(ENABLEDEPTH)
        {
            int text_line2_x1 = text_line1_x1;
            int text_line2_y1 = (int)(text_line1_y1 + text2.height + baseline/2 + 1);
            cv::putText(img, label2, cv::Point(text_line2_x1, text_line2_y1), fontface, scale, cv::Scalar(0, 0, 255), thickness, cv::LINE_AA); 
        }

        cv::rectangle(img, Rect(x1, y1, x2, y2), colors, 2);
    }
    
    putText(img, format("num: %d", no_of_persons), Point(0, 30), 0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
}

int main( int argc, char** argv )
{
    //Handle system signals
    signal(SIGINT, signalHandler);

    bool enableDepth = false;
    if(argc>1)
    {
        int input = atoi(argv[1]);
        enableDepth = (input>0) ? true: false;
    }

    // Port used for UART connection can be differnt for different carrier boards. Get the port name for the carrier board being used.
    string port_name = getPortName();

    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    int serial_port = open(port_name.c_str(), O_RDWR);

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

    const char* Start = "S"; // Mark start of the frame
    const char* End = "E";   // Mark end of the frame
    const char* new_frame = "D"; // Start of new index 
    const char* Check = "C"; // Mark Checksum 

    //camera index is the video index assigned by the system to the camera. 
    //By default we set it to 5. Specify the index if the value has been changed.
    std::vector<int> camera_indexes{5};
    
    //Connect to the PAL camera
    if (PAL::Init(camera_indexes) != PAL::SUCCESS) 
    {
        cerr<<"Init failed"<<endl;
        return 1;
    }

    //Setting API Mode
    PAL::SetAPIMode(PAL::API_Mode::TRACKING);
    
    //Loading camera properties from a text file
    PAL::CameraProperties properties;
    PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedProperties.yml", &properties);
    if(ack_load == PAL::Acknowledgement::INVALID_PROPERTY_VALUE)
    {
        PAL::Destroy();
        return 1;
    }
    if(ack_load != PAL::SUCCESS)
    {
        cerr<<"Error Loading settings! Loading default values."<<endl;
    }
    
    //Set in which mode to run tracking
    int tracking_mode = PAL::Tracking_Mode::PEOPLE_TRACKING;
    int success = PAL::SetModeInTracking(tracking_mode);

    //Set depth detection mode
    unsigned long int flags = PAL::DEPTH_IN_TRACKING;
    if(enableDepth)
    {
        properties.depth_in_tracking = PAL::DepthInTracking::DEPTH_ON;
    }
    else
    {
        properties.depth_in_tracking = PAL::DepthInTracking::DEPTH_OFF;
    }
    PAL::SetCameraProperties(&properties, &flags);

    // Create a window for display.
    namedWindow("PAL OBJECT_TRACKING_UART", WINDOW_AUTOSIZE);

    std::cout << "Press CTRL+C to close the application." << std::endl;

    int key = ' ';

    while(!g_bExit)
    {
        std::vector<PAL::Data::Tracking_Data> data;
        data =  PAL::GrabTrackingData();
        if(data[0].camera_changed)
        {
            //exiting application when camera is changed
            break;
        }

        cv::Mat display = data[0].left;
        drawOnImage(display, data[0], tracking_mode, enableDepth, false);
        
        int person_detected = data[0].tracking_info[PAL::States::OK].size();
        if(person_detected)
        {
            // Writing Start of the frame to the serial port
            write(serial_port, Start, 1);    

            cout<<"[INFO] START OF NEW FRAME"<<endl;
            
            char num_buffer[int_length(person_detected)]; 
            my_itoa(person_detected, num_buffer);
            write(serial_port, num_buffer, sizeof(num_buffer));
            
            cout<<"Num of person detected: " << person_detected << endl;
            
            //Converting numerical depth values to string and writing them to uart port
            if(enableDepth)
            {
                for(int i=0; i<person_detected; i++)
                {
                    //validating depth values
                    bool invalid = std::isnan(depthValues[i]) || (depthValues[i]<0) || (depthValues[i]>1000);                    
                    depthValues[i] = (invalid) ? 100: depthValues[i];
                }

                for(int i=0; i<person_detected; i++)
                {
                    //converting numerical values
                    char buffer[int_length(depthValues[i])]; 
                    my_itoa(depthValues[i], buffer);
                    cout << " DISTANCE : " << (int)depthValues[i] << " cm" << endl;

                    //publishing them over port
                    write(serial_port, new_frame, 1);
                    write(serial_port, buffer, sizeof(buffer));
                }
            }
            // Writing End of the frame to the serial port                        
            write(serial_port, End, 1);                            
            cout << "[INFO] END OF THE FRAME" << endl << endl;
        }  
    }

    cout << "exiting the application" << endl;
    PAL::Destroy();
   
    return 0;
}
