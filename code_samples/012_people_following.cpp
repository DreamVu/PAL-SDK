/*

   CODE SAMPLE # 011: Object Tracking
   This code will grab the basic stereo panoramas (left and right images) and ALSO the Disparity panorama, execute tracking for objects on it and then display in an opencv window


   >>>>>> Compile this code using the following command....


   g++ 011_object_tracking.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DE.so libPAL_Track.so `pkg-config --libs --cflags opencv`   -g  -o 011_object_tracking.out -I../include/ -I/usr/local/include/eigen3     -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl


   >>>>>> Execute the binary file by typing the following command...


   ./011_object_tracking.out


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

using namespace cv;
using namespace std;

using namespace std::chrono;

namespace PAL
{

	int RunTrack(cv::Mat& img, cv::Mat& depth, vector<vector<float>> &boxes, 
			vector<int> &ids, vector<float> &depthValues, vector<Scalar> &colours);
}

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


int main(int argc, char *argv[])
{

	namedWindow("Pal Object Tracking", WINDOW_NORMAL); // Create a window for display.

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
	flag = flag | PAL::CAMERA_HEIGHT;

	prop.mode = PAL::Mode::TRACKING;
	prop.fd = 1;
	prop.nr = 0;
	prop.filter_spots = 1;
	prop.vertical_flip =0;
	prop.camera_height = 60;
	PAL::SetCameraProperties(&prop, &flag);

	bool isDisparityNormalized = true;

	bool followFlag = true, followFound = false;
	int followID = 0;
	int ftop,fleft,fwidth,fheight;
	int prevPad = 0;

	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	resizeWindow("Pal Object Tracking", width / 4, (height / 4) * 3);

	int key = ' ';

	printf("Press ESC to close the window\n");
	printf("Press v/V key to toggle the vertical flip of panorama\n");
	printf("Press f/F to toggle filter rgb property.\n");
	printf("Press d/D to toggle fast depth property\n");
	printf("Press r/R to toggle near range property\n");

	size_t currentResolution = 0;

	bool flip = false;
	bool filter_spots = true;
	bool nr = false;
	bool fd = true;

	vector<vector<float>> boxes; 
	vector<int> ids; 
	vector<float> depthValues; 
	vector<Scalar> colours; 
	int num;

	//27 = esc key. Run the loop until the ESC key is pressed
	while (key != 27)
	{
		PAL::Image left, right, depth, disparity;
		//PAL::GrabFrames(&left, &right, &depth);
		PAL::GrabFrames(&left, &right);
		//Convert PAL::Image to Mat
		Mat img0 = Mat(left.rows, left.cols, CV_8UC3, left.Raw.u8_data);
                Mat img;// = img0.clone();

		if(followFlag && followFound)
		{
		        if(prevPad-544<0)
		        {
		                int leftlen = prevPad-544;
		                int rightlen = prevPad+544;
		                //cout<<"Pan-Left::"<<leftlen<<"::"<<rightlen<<endl;
		                cv::Rect lroi(1120+leftlen,0,abs(leftlen), 384);
		                cv::Rect rroi(0,0,rightlen,384);
		                hconcat(img0(lroi),img0(rroi),img);
		        }
		        else if(prevPad+544>1120)
			{
			        int leftlen = 1120-(prevPad-544);
		                int rightlen = (prevPad+544)-1120;
		                //cout<<"Pan-Right::"<<leftlen<<"::"<<rightlen<<endl;
		                cv::Rect lroi(1120-leftlen,0,abs(leftlen), 384);
		                cv::Rect rroi(0,0,rightlen,384);
		                hconcat(img0(lroi),img0(rroi),img);			        
			}
			else
			{
			        int leftStart = prevPad-544;
			        int rightEnd = prevPad + 544;
				//cout<<"Within-Limits::"<<leftStart<<"::"<<rightEnd<<endl;
				cv::Rect roi(leftStart, 0, 1088, 384);
			        img = img0(roi);
			}
		}
		else
		{
			cv::Rect roi(16, 0, 1088, 384);
			img = img0(roi);
			prevPad = 560; 
		}

		//Mat d = Mat(depth.rows, depth.cols, CV_32FC1, depth.Raw.f32_data);
		Mat d = cv::Mat::zeros(cv::Size(1, 1), CV_32FC1);
		num = PAL::RunTrack(img, d, boxes, ids, depthValues, colours);

		for (int i = 0; i < boxes.size(); i++)
		{
			//putText(img, format("ID=%d , Depth=%.2fm", ids[i], depthValues[i]/100), Point(boxes[i][0], boxes[i][1]+25+boxes[i][3]), 
			//         0, 0.6, Scalar(0, 0, 255), 1, LINE_AA);
			putText(img, format("ID=%d", ids[i]), Point(boxes[i][0], boxes[i][1]+25+boxes[i][3]), 0, 0.6, Scalar(0, 0, 255), 1, LINE_AA);
			//putText(img, format("ID=%d , Depth=%.2fm", ids[i], depthValues[i]/100), Point(boxes[i][0], boxes[i][1] - 5), 
			//0, 0.6, Scalar(0, 0, 255), 1, LINE_AA);
			rectangle(img, Rect(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3]), colours[i], 2);
		}
		putText(img, format("num: %d", num), Point(0, 30), 0, 0.6, Scalar(0, 0, 255), 2, LINE_AA);

		if(followFlag)
		{
			for (int i=0; i<boxes.size(); i++)
			{
				if(ids[i]==followID)
				{
					fleft=boxes[i][0];
					ftop=boxes[i][1];
					fwidth=boxes[i][2];
					fheight=boxes[i][3]; 
					//cout<<"T:"<<ftop<<"L:"<<fleft<<"W:"<<fwidth<<"H:"<<fheight<<endl;
					followFound=true;
					//cout<<"PrevPad::"<<prevPad<<endl;
					prevPad = prevPad - 544 + fleft + fwidth/2;
					prevPad = prevPad%1120;
					//cout<<"PrevPad::"<<prevPad<<endl;
					break;
				}

			}
		}
		//Display the final output image
		imshow("Pal Object Tracking", img);

		boxes.clear();
		ids.clear();
		depthValues.clear();
		colours.clear();

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;

                if (key == 'i' || key == 'I')
		{
			string zenityCmd = "zenity --entry --text \"Enter ID to Track\" --title \"PAL People Following\" --entry-text=\"\"";
                        string output = getCmdOutput(zenityCmd);
                        followID = stoi(output);
		}

		if (key == 'v' || key == 'V')
		{
			PAL::CameraProperties prop;
			flip = !flip;
			prop.vertical_flip = flip;
			unsigned int flags = PAL::VERTICAL_FLIP;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if (key == 'f' || key == 'F')
		{	
			PAL::CameraProperties prop;
			filter_spots = !filter_spots;
			prop.filter_spots = filter_spots;
			unsigned int flags = PAL::FILTER_SPOTS;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if(key == 'd' || key == 'D')
		{
			PAL::CameraProperties prop;
			fd = !fd;
			prop.fd = fd;
			unsigned int flags = PAL::FD;
			PAL::SetCameraProperties(&prop, &flags);
		}
		if(key == 'r' || key == 'R')
		{		
			PAL::CameraProperties prop;
			nr = !nr;
			prop.nr = nr;
			unsigned int flags = PAL::NR;
			PAL::SetCameraProperties(&prop, &flags);
		}
	}
	printf("exiting the application\n");
	//CloseTrack();
	PAL::Destroy();

	return 0;
}
