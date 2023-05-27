/*

CODE SAMPLE # 016: Multi Zone Detection
This code will grab the left panorama with person tracking data and check if they are in the user provided distinctive detection zones

>>>>>> Compile this code using the following command....

./compile.sh 016_multi_zone_detection.cpp

>>>>>> Execute the binary file by typing the following command...

./016_multi_zone_detection.out
      
*/

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "PAL.h"

using namespace cv;
using namespace std;

std::string precision_string(float num, int precision=1)
{
    std::string num_string = std::to_string(num);
    return num_string.substr(0, num_string.find(".")+1+precision);
}

void zoneDetection(cv::Mat &img, std::vector<std::vector<PAL::Data::TrackND>> detections, vector<float> &thresh)
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

    bool only_detection = false;
    bool ENABLEDEPTH = true;
    bool ENABLE3D = true;

    int no_of_persons = detections[PAL::States::OK].size();

    std::cout << "-----------------------------------------------" << std::endl;
    for (int i = 0; i < no_of_persons; i++)
    {
        int x1,y1,x2,y2;
        x1 = (int)detections[PAL::States::OK][i].boxes.x1;
        y1 = (int)detections[PAL::States::OK][i].boxes.y1;
        x2 = (int)detections[PAL::States::OK][i].boxes.x2;
        y2 = (int)detections[PAL::States::OK][i].boxes.y2;

        float x3D, y3D, z3D, depth_value;
        x3D = detections[PAL::States::OK][i].locations_3d.x;
        y3D = detections[PAL::States::OK][i].locations_3d.y;
        z3D = detections[PAL::States::OK][i].locations_3d.z;

        depth_value = sqrt(x3D*x3D + y3D*y3D);

        int fontface = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 0.4;
        int thickness = 1;
        int baseline = 0;
        int height_mul = 3;

        std::string label1, label2;
        cv::Size text1, text2;
        
        if(only_detection)
        {
            label1 = "Class= " + classes[ round(detections[PAL::States::OK][i].t_label) ];
        }
        else
        {
            label1 = "ID=" + to_string((int)detections[PAL::States::OK][i].t_track_id) + 
                ", "+classes[ round(detections[PAL::States::OK][i].t_label) ];
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

        cv::Scalar colors;
        int number_of_zones = thresh.size();
        bool beyond_all_zones = true;
        for(int j=0 ; j<number_of_zones ; j++)
        {
            if(depth_value < thresh[j])
            {
                std::cout << "ID " << to_string((int)detections[PAL::States::OK][i].t_track_id) << " belongs to zone " << (j+1) << std::endl;
                float normalised_color = (float)(j/(float)(number_of_zones)) ;            
                int green = (int)(255.0f*normalised_color);
                int red = (int)(255.0f*(1.0f-normalised_color));
                colors = cv::Scalar(0, green, red);
                beyond_all_zones = false;
                break;
            }
        }

        if(beyond_all_zones)
        {
            std::cout << "ID " << to_string((int)detections[PAL::States::OK][i].t_track_id) << " belongs beyond zone " << number_of_zones << std::endl;
            colors = cv::Scalar(0, 255, 0);
        }

        cv::putText(img, label1, Point(x1, (int)(y1-text2.height-baseline-2)), fontface, scale, colors, thickness, cv::LINE_AA);
        if(ENABLEDEPTH)
        {
            cv::putText(img, label2, Point(x1, (int)(y1-baseline/2 -1)), fontface, scale, colors, thickness, cv::LINE_AA); 
        }

            
        cv::rectangle(img, Rect(x1, y1, x2, y2), colors, 2);
    }
    
    putText(img, format("num: %d", no_of_persons), Point(0, 30), 0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    std::cout << "-----------------------------------------------" << std::endl << std::endl;
}

void print_track(std::vector<std::vector<PAL::Data::TrackND>> results)
{
    if(results.size())
    {
        for(int i = 0; i < results[PAL::States::OK].size(); i++)
        {
            cout << " in OK TrackID: " << results[PAL::States::OK][i].t_track_id << endl;
            cout << " in OK Activated: " << results[PAL::States::OK][i].t_is_activated << endl;
            cout << " in OK Activate: " << results[PAL::States::OK][i].active << endl;
            cout << " in OK x: " << results[PAL::States::OK][i].locations_3d.x << endl;
            cout << " in OK y: " << results[PAL::States::OK][i].locations_3d.y << endl;
            cout << " in OK z: " << results[PAL::States::OK][i].locations_3d.z << endl;
            cout << " in OK Score  : " << results[PAL::States::OK][i].t_score << endl;
            cout << " in OK Label  : " << results[PAL::States::OK][i].t_label << endl;
            cout << " in OK boxes x1  : " << results[PAL::States::OK][i].boxes.x1 << endl;
            cout << " in OK boxes y1  : " << results[PAL::States::OK][i].boxes.y1 << endl;
            cout << " in OK boxes x2  : " << results[PAL::States::OK][i].boxes.x2 << endl;
            cout << " in OK boxes y2  : " << results[PAL::States::OK][i].boxes.y2 << endl;
        }
        cout << endl;
    }    
    if(results.size()>1)
    {
        for(int i = 0; i < results[PAL::States::SEARCHING].size(); i++)
        {
            cout << " in SEARCHING TrackID: " << results[PAL::States::SEARCHING][i].t_track_id << endl;
            cout << " in SEARCHING Activated: " << results[PAL::States::SEARCHING][i].t_is_activated << endl;
            cout << " in SEARCHING Activate: " << results[PAL::States::SEARCHING][i].active << endl;
            cout << " in SEARCHING x: " << results[PAL::States::SEARCHING][i].locations_3d.x << endl;
            cout << " in SEARCHING y: " << results[PAL::States::SEARCHING][i].locations_3d.y << endl;
            cout << " in SEARCHING z: " << results[PAL::States::SEARCHING][i].locations_3d.z << endl;
            cout << " in SEARCHING Score  : " << results[PAL::States::SEARCHING][i].t_score << endl;
            cout << " in SEARCHING Label  : " << results[PAL::States::SEARCHING][i].t_label << endl;
            cout << " in SEARCHING boxes x1  : " << results[PAL::States::SEARCHING][i].boxes.x1 << endl;
            cout << " in SEARCHING boxes y1  : " << results[PAL::States::SEARCHING][i].boxes.y1 << endl;
            cout << " in SEARCHING boxes x2  : " << results[PAL::States::SEARCHING][i].boxes.x2 << endl;
            cout << " in SEARCHING boxes y2  : " << results[PAL::States::SEARCHING][i].boxes.y2 << endl;
        }
        cout << endl;
        for(int i = 0; i < results[PAL::States::TERMINATED].size(); i++)
        {
            cout << " in TERMINATED TrackID: " << results[PAL::States::TERMINATED][i].t_track_id << endl;
            cout << " in TERMINATED Activated: " << results[PAL::States::TERMINATED][i].t_is_activated << endl;
            cout << " in TERMINATED Activate: " << results[PAL::States::TERMINATED][i].active << endl;
            cout << " in TERMINATED x: " << results[PAL::States::TERMINATED][i].locations_3d.x << endl;
            cout << " in TERMINATED y: " << results[PAL::States::TERMINATED][i].locations_3d.y << endl;
            cout << " in TERMINATED z: " << results[PAL::States::TERMINATED][i].locations_3d.z << endl;
            cout << " in TERMINATED Score  : " << results[PAL::States::TERMINATED][i].t_score << endl;
            cout << " in TERMINATED Label  : " << results[PAL::States::TERMINATED][i].t_label << endl;
            cout << " in TERMINATED boxes x1  : " << results[PAL::States::TERMINATED][i].boxes.x1 << endl;
            cout << " in TERMINATED boxes y1  : " << results[PAL::States::TERMINATED][i].boxes.y1 << endl;
            cout << " in TERMINATED boxes x2  : " << results[PAL::States::TERMINATED][i].boxes.x2 << endl;
            cout << " in TERMINATED boxes y2  : " << results[PAL::States::TERMINATED][i].boxes.y2 << endl;
        }
        cout << endl;
    }

    cout << endl;
}

int main( int argc, char** argv )
{
    //provide zone range as command line arguments
    if ((argc < 2) || (stoi(argv[1]) != (argc - 2)))
    {
        std::cout << "Wrong format for arguments" << std::endl;
        std::cout << "Expected format:" << std::endl;
        std::cout << "./016_multi_zone_detection.out <video index> <number of zones> <distance 1> <distance 2> .. <distance for last zone>" << std::endl;
        return 1;
    }

    //Assign the given zone distances
    int number_of_zones = stoi(argv[1]);
    
    vector <float> distances(number_of_zones);
    for(int i=0 ; i<number_of_zones ; i++)
    {
        distances[i] = stof(argv[i+2])/100.0f;
    }

    sort(distances.begin(), distances.end());

    //Camera index is the video index assigned by the system to the camera. 
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

    //Set depth detection mode
    PAL::SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_3DLOCATION_ON);

    //Set in which mode to run tracking
    int tracking_mode = PAL::Tracking_Mode::PEOPLE_TRACKING;
    int success = PAL::SetModeInTracking(tracking_mode);

    // Create a window for display.
    namedWindow( "PAL Multi Zone Detection", WINDOW_AUTOSIZE);

    cout << "Press ESC to close the window." << endl;
    cout << "Press f/F to toggle filter rgb property" << endl;
    cout << "Press v/V to toggle Vertical Flip property." << endl;
    cout << "Press m/M to toggle Fast Depth property" << endl;

    std::vector<PAL::Data::TrackingResults> data;

    int key = ' ';

    do
    {
        //Capturing Detection data from the camera
        data =  PAL::GrabTrackingData();    

        cv::Mat display = data[0].left;
        
        zoneDetection(display, data[0].trackingData, distances);
        
        //Display the stereo images
        imshow( "PAL Multi Zone Detection", display);  

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
        if (key == 'm' || key == 'M')
        {           
            properties.fd = !properties.fd;
            unsigned long int flags = PAL::FD;
            PAL::SetCameraProperties(&properties, &flags);
        }
    }
    //27 = esc key. Run the loop until the ESC key is pressed and camera is not changed
    while(key != 27 && !data[0].camera_changed);

    cout<<"exiting the application"<<endl;
    PAL::Destroy();
   
    return 0;
}
