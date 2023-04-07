/*

CODE SAMPLE # 009: Object Tracking
This code will grab the left panorama with object tracking data overlayed on it and would be displayed in a window using opencv



>>>>>> Compile this code using the following command....

./compile.sh 009_object_tracking.cpp


>>>>>> Execute the binary file by typing the following command...

./009_object_tracking.out


>>>>>> KEYBOARD CONTROLS:

    Press ESC to close the window.
    Press f/F to toggle filter rgb property
    Press v/V to toggle Vertical Flip property.
    Press d/D to enable/Disable Depth calculation.
    Press l/L to enable/Disable 3D Location calculation.
    Press m/M to toggle Fast Depth property.
*/


# include <stdio.h>

# include <opencv2/opencv.hpp>

# include "PAL.h"
#include "TimeLogger.h"
#include <time.h>
#include <unistd.h>

#include<sys/time.h>

#include <iomanip>

using namespace cv;
using namespace std;

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

    bool only_detection = (mode == PAL::Tracking_Mode::PEOPLE_DETECTION || mode == PAL::Tracking_Mode::OBJECT_DETECTION) ? true : false;
    if(!ENABLEDEPTH)
        ENABLE3D = false;

    int no_of_persons = data.trackingData[PAL::States::OK].size();
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
    namedWindow( "PAL OBJECT_TRACKING", WINDOW_NORMAL ); // Create a window for display.

    //Select the Model to use in Tracking. To be set before Init call.
    PAL::SetInitTrackingModel(PAL::Tracking_Model::MODEL_1);

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

    PAL::CameraProperties cam_data;
    PAL::Acknowledgement ack_load = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &cam_data);

    if(ack_load != PAL::SUCCESS)
    {
        cout<<"Error Loading settings! Loading default values."<<endl;
    }

    bool filter_spots = cam_data.filter_spots;
    bool flip = cam_data.vertical_flip;
    bool fd = cam_data.fd;

    bool enableDepth = false;
    bool enable3Dlocation = false;
    PAL::SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_OFF);

    int tracking_mode = PAL::Tracking_Mode::OBJECT_TRACKING;
    int success = PAL::SetModeInTracking(tracking_mode);
	std::vector<PAL::Data::TrackingResults> dataDiscard;
	dataDiscard =  PAL::GrabTrackingData();    

	width = dataDiscard[0].left.cols;
	height = dataDiscard[0].left.rows;

    //width and height are the dimensions of each panorama.
    //Each of the panoramas are displayed at otheir original resolution.
    resizeWindow("PAL OBJECT_TRACKING", width, height);

    int key = ' ';

    cout << "Press ESC to close the window." << endl;
    cout << "Press f/F to toggle filter rgb property" << endl;
    cout << "Press v/V to toggle Vertical Flip property." << endl;
    cout << "Press d/D to enable/Disable Depth calculation." << endl;
    cout << "Press l/L to enable/Disable 3D Location calculation." << endl;
    cout << "Press m/M to toggle Fast Depth property" << endl;

	
	//27 = esc key. Run the loop until the ESC key is pressed
	while(key != 27)
	{


        std::vector<PAL::Data::TrackingResults> data;
        data =  PAL::GrabTrackingData();    
		if(data[0].camera_changed)
		{
			break;
		}
        cv::Mat display = data[0].left;
        drawOnImage(display, data[0], tracking_mode, enableDepth, enable3Dlocation);
        
        //Display the stereo images
        imshow( "PAL OBJECT_TRACKING", display);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;
        
        if (key == 'f' || key == 'F')
        {   
            PAL::CameraProperties prop;
            filter_spots = !filter_spots;
            prop.filter_spots = filter_spots;
            unsigned long int flags = PAL::FILTER_SPOTS;
            PAL::SetCameraProperties(&prop, &flags);
        }

        if (key == 'v' || key == 'V')
        {           
            PAL::CameraProperties prop;
            flip = !flip;
            prop.vertical_flip = flip;
            unsigned long int flags = PAL::VERTICAL_FLIP;
            PAL::SetCameraProperties(&prop, &flags);
        }

        if (key == 'm' || key == 'M')
        {           
            PAL::CameraProperties prop;
            fd = !fd;
            prop.fd = fd;
            unsigned long int flags = PAL::FD;
            PAL::SetCameraProperties(&prop, &flags);
        }

        if (key == 'd' || key == 'D')
        {   
            enableDepth = !enableDepth;
            if(enableDepth)
            {
                if(enable3Dlocation)
                    SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_3DLOCATION_ON);
                else
                    SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_ON);
            }
            else
                SetDepthModeInTracking(PAL::DepthInTracking::DEPTH_OFF);
        }

        if (key == 'l' || key == 'L')
        {
            enable3Dlocation = !enable3Dlocation;
        }
    }

    printf("exiting the application\n");
    PAL::Destroy();
   
    return 0;
}

