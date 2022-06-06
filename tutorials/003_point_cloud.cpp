/*

PAL TUTORIAL # 004: Compute the point cloud and save it into a file

This tutorial shows the minimal code required to...
1. Compute the point cloud - based on the stereo panoramas
2. Save the point cloud into a file, that can be opened with tools like blender, meshlab etc.


Compile this file using the following command....
g++ 003_point_cloud.cpp  /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so   ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DE.so ../lib/libPAL_EDET.so `pkg-config --libs --cflags opencv`   -lv4l2 -lpthread -O3 -o 003_point_cloud.out -I../include/


Run the output file using the following command....
./004_point_cloud.out
*/

# include <stdio.h>s
# include <unistd.h>
# include <sys/types.h>

# include "PAL.h"

int main(int argc, char** argv)
{
	int image_width = -1;
	int image_height = -1;

    
    if(PAL::Init(image_width, image_height,-1) != PAL::SUCCESS)
        return 1; //Init failed

    PAL::CameraProperties data; 
    PAL::Acknowledgement ack = PAL::LoadProperties("../../Explorer/SavedPalProperties.txt", &data);
    if(ack != PAL::SUCCESS)
    {
        printf("Error Loading settings\n");
    }
    PAL::CameraProperties prop;
    unsigned int flag = PAL::MODE | PAL::DISPARITY_COMPUTATION | PAL::VERTICAL_FLIP | PAL::FD;
    prop.mode = PAL::Mode::POINT_CLOUD_25D; // The other available option is PAL::Mode::HIGH_QUALITY_DEPTH
    prop.computation = PAL::HIGH_QUALITY_B;
    prop.vertical_flip=0;
    prop.fd = 1;
    PAL::SetCameraProperties(&prop, &flag);
        
    for(int i=0; i<10;i++)
    {
	    std::vector<PAL::Point> pc;
	    if (PAL::GetPointCloud(&pc) == PAL::SUCCESS)
	    {
		    printf("saving the point cloud\n");
		    PAL::SavePointCloud("point_cloud.ply", &pc);
	    }
	    else
	    {
		    printf("unable to compute the point cloud\n");
	    }
    }
    
    PAL::Destroy();
	
	return 0;

}
