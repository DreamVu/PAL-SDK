/*

CODE SAMPLE # 010: Object following
This code sample will center on the object being tracked and follow it

>>>>>> Compile this code using the following command....

./compile.sh 010_object_following.cpp

>>>>>> Execute the binary file by typing the following command...

./010_object_following.out

*/

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "PAL.h"

using namespace cv;
using namespace std;

string getCmdOutput(string cmd)
{
   string outputString;
   FILE *outpStream;
   const int MaxLineLen = 128;
   char  outpLine[MaxLineLen];
   outpStream = popen(cmd.c_str(), "r");
   if (outpStream) {
      while (!feof(outpStream)) {
          if (fgets(outpLine, MaxLineLen, outpStream) != NULL) {
             outputString += outpLine;
          }
      }
      pclose(outpStream);
   }
   return outputString;
}

bool is_number(const std::string& s)
{
    return !s.empty() && std::find_if(s.begin(), 
        s.end(), [](unsigned char c) { return !std::isdigit(c); }) == s.end();
}

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
    PAL::CameraProperties *properties=nullptr)
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

    bool only_detection = (mode == PAL::Tracking_Mode::PEOPLE_DETECTION || 
                           mode == PAL::Tracking_Mode::OBJECT_DETECTION) ? true : false;
    bool ENABLEDEPTH = (properties->depth_in_tracking == PAL::DepthInTracking::DEPTH_ON || 
                        properties->depth_in_tracking == PAL::DepthInTracking::DEPTH_3DLOCATION_ON);
    bool ENABLE3D = properties->depth_in_tracking == PAL::DepthInTracking::DEPTH_3DLOCATION_ON;

    int no_of_persons = data.tracking_info[PAL::States::OK].size();
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
    int tracking_mode = PAL::Tracking_Mode::OBJECT_FOLLOWING;
    int success = PAL::SetModeInTracking(tracking_mode);

    // Create a window for display.
    namedWindow( "PAL Object Following", WINDOW_AUTOSIZE);

    cout << "Press ESC to close the window." << endl;
    cout << "Press f/F to toggle filter rgb property" << endl;
    cout << "Press v/V to toggle Vertical Flip property." << endl;
    cout << "Press z/Z to Disable Depth calculation." << endl;
    cout << "Press x/X to Enable Depth calculation." << endl;
    cout << "Press c/C to Enable 3D Location calculation." << endl;
    cout << "Press m/M to toggle Fast Depth property." << endl;
    cout << "Press i/I to set ID of the object you want to follow." << endl;
    cout << "Press p/P to print the ID of the object being followed." << endl;

    std::vector<PAL::Data::Tracking_Data> data;

    int key = ' ';

    do
    {
        //Capturing Tracking data from the camera
        data =  PAL::GrabTrackingData();    

        cv::Mat display = data[0].left;
        drawOnImage(display, data[0], tracking_mode, &properties);
        
        //Display the stereo images
        imshow( "PAL Object Following", display);  

        //Wait for the keypress - with a timeout of 1 ms
        key = waitKey(1) & 255;

        //Take ID from user using a zenity popup window and start following that ID
        if (key == 'i' || key == 'I')
        {
            string zenityCmd = "zenity --entry --text \"Enter ID to Track\" --title \"PAL Object Following\" --entry-text=\"\"";
            string output = getCmdOutput(zenityCmd);

            output.erase(std::remove(output.begin(), output.end(), ' '), output.end());
            output.erase(std::remove(output.begin(), output.end(), '\n'), output.end());
            output.erase(std::remove(output.begin(), output.end(), '\t'), output.end());

            if(is_number(output))
            {
                PAL::SetTrackID(stoi(output));
            }    
            else
            {
                string errorCmd = "zenity --warning  --text \"Not a valid input\" --title \"PAL Object Following\"";
                FILE *outpStream = popen(errorCmd.c_str(), "r");
                pclose(outpStream);      
            }
        }
        if (key == 'p' || key == 'P')
        {
            int new_id = PAL::GetTrackID();
            if(new_id == -2)
            {
                std::cout << "Currently not in Following Mode" << std::endl;
            }
            else if(new_id == -1)
            {
                std::cout << "No ID has been entered. Press 'i' to enter an ID" << std::endl; 
            }
            else
            {
                std::cout << "The ID currently being followed is: " << new_id << std::endl;
            }
        }
        
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

        if (key == 'z' || key == 'Z')
        {   
            properties.depth_in_tracking = PAL::DepthInTracking::DEPTH_OFF;
            unsigned long int flags = PAL::DEPTH_IN_TRACKING;
            PAL::SetCameraProperties(&properties, &flags);
        }
        
        if (key == 'x' || key == 'X')
        {   
            properties.depth_in_tracking = PAL::DepthInTracking::DEPTH_ON;
            unsigned long int flags = PAL::DEPTH_IN_TRACKING;
            PAL::SetCameraProperties(&properties, &flags);
        }

        if (key == 'c' || key == 'C')
        {   
            properties.depth_in_tracking = PAL::DepthInTracking::DEPTH_3DLOCATION_ON;
            unsigned long int flags = PAL::DEPTH_IN_TRACKING;
            PAL::SetCameraProperties(&properties, &flags);
        }
    }
    //27 = esc key. Run the loop until the ESC key is pressed and camera is not changed
    while(key != 27 && !data[0].camera_changed);

    cout<<"exiting the application"<<endl;
    PAL::Destroy();

    return 0;
}
