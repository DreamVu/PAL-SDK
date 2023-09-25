/*

CODE SAMPLE # 006: Occupancy Map
This code sample allows users to access the region map within a depth range.

>>>>>> Compile this code using the following command....

./compile.sh 006_occupancy_map.cpp

>>>>>> Execute the binary file by typing the following command...

./006_occupancy_map.out

*/

#include <stdio.h>
#include <limits>
#include <opencv2/opencv.hpp>
#include "PAL.h"

using namespace cv;
using namespace std;

template <typename T> void GetUserInput(std::string text, T &value)
{
    while (true) 
    {
        std::cout << text << std::endl;
        if (std::cin >> value) 
        {
            if (std::cin.peek() == '\n')
            {
                break;
            }
        }

        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input. Please enter a valid value." << std::endl << std::endl;
    }
    return;
}

cv::Mat GetOccupancy1D(cv::Size rgb_size, Mat depth_mat, int depth_thresh, float context_threshold) 
{
    cv::Mat mask;
    
    //set values closer than the provided depth threshold as 255. 
    cv::threshold(depth_mat, mask, depth_thresh, 255, cv::THRESH_BINARY_INV);

    //make a binary mask of value 0/1.
    mask = mask/255;

    //create a row mat with sum of each column
    cv::Mat occupancySum;
    cv::reduce(mask, occupancySum, 0, cv::REDUCE_SUM, CV_32FC1);

    //check which columns fell above our context threshold
    cv::Mat occupancy1D; 
    cv::threshold(occupancySum, occupancy1D, rgb_size.height*context_threshold/100.0, 255, cv::THRESH_BINARY_INV);

    //create a 0/1 mask with 1 meaning that that column is occupied
    occupancy1D = occupancy1D/255;
    occupancy1D.convertTo(occupancy1D, CV_8UC1);        

    return occupancy1D;
}

cv::Mat GetColorMap(cv::Size rgb_size, cv::Mat &occupancy1D)
{
    cv::Mat r = cv::Mat::ones(1, rgb_size.width, CV_8UC1);
    cv::Mat g = cv::Mat::ones(1, rgb_size.width, CV_8UC1);
    cv::Mat b = cv::Mat::ones(1, rgb_size.width, CV_8UC1);

    unsigned char* pt_occupancy1D = (unsigned char* )occupancy1D.data;
    unsigned char* pt_r = (unsigned char* )r.data;

    //double the value of r channel when the column is occupied
    for(int i=0; i<rgb_size.width; i++,pt_occupancy1D++,pt_r++)
    {
        if(!(*pt_occupancy1D))
        {
            *pt_r = (*pt_r)*2;
        }
    }
    
    vector<Mat> channels;
    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);

    cv::Mat color_map;
    merge(channels, color_map);
    
    //convert the 1D map to 2D
    resize(color_map, color_map, rgb_size);

    return color_map;
}

int main(int argc, char *argv[])
{
    //depth threshold in cm
    //The depth threshold should be kept within 1m to 2m range.
    int threshold_cm;
    std::string threshold_msg = "Enter the depth threshold in cm, eg 100";
    GetUserInput<int>(threshold_msg, threshold_cm);

    if (threshold_cm > 200)
    {
        threshold_cm = 200;
        cout<<"depth threshold set above maximum range. Setting to 2m"<<endl;
    }
    else if (threshold_cm < 100)
    {
        threshold_cm = 100;
        cout<<"depth threshold set below minimum range. Setting to 1m"<<endl;
    }

    //context threshold in percentage to be considered for occupancy
    //The context threshold should be kept within 50(recommended) to 80 range.
    int context_threshold;
    std::string context_threshold_msg = "Enter the Context threshold. It is the percentage to be considered for occupancy, eg 50";
    GetUserInput<int>(context_threshold_msg, context_threshold);

    if (context_threshold > 80)
    {
        context_threshold = 80;
        cout<<"context threshold set above maximum range. Setting to 80"<<endl;
    }
    else if(context_threshold < 50)
    {
        context_threshold = 50;
        cout<<"context threshold set below miminum range. Setting to 50"<<endl;
    }
    
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
    namedWindow( "PAL Occupancy Map", WINDOW_AUTOSIZE);

    cout<<endl;
    cout<<"Press ESC to close the window"<<endl; 
    cout<<"Press v/V to toggle vertical flip property"<<endl; 
    cout<<"Press f/F to toggle filter rgb property"<<endl;

    std::vector<PAL::Data::ODOA_Data> data;

    int key = ' ';

    do
    {
        //Capturing Depth data from the camera
        data =  PAL::GrabRangeScanData();

        cv::Mat left = data[0].left.clone();
        
        Mat depth;
        if(properties.raw_depth)
            depth = data[0].raw_depth.clone();
        else
            depth = data[0].depth.clone();  
        depth.convertTo(depth, CV_8UC1);

        //Get a 1D occupancy mask
        cv::Mat occupancy1D = GetOccupancy1D(left.size(), depth, threshold_cm, context_threshold);

        //Get a 2D color occupancy mask
        cv::Mat color_mask = GetColorMap(left.size(), occupancy1D);
        
        //Apply the color mask
        cv::Mat display = left.mul(color_mask);
        
        imshow("PAL Occupancy Map", display);
        
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
    }
    //27 = esc key. Run the loop until the ESC key is pressed and camera is not changed
    while(key != 27 && !data[0].camera_changed);

    cout<<"exiting the application"<<endl;
    PAL::Destroy();
    
    return 0;
}
