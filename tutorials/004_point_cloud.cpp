/*

PAL TUTORIAL # 004: Compute the point cloud and save it into a file

This tutorial shows the minimal code required to...
1. Compute the point cloud - based on the stereo panoramas
2. Save the point cloud into a file, that can be opened with tools like blender, meshlab etc.

g++ 004_point_cloud.cpp ../lib/libPAL.so `pkg-config --libs --cflags opencv`   -lv4l2 -lpthread -g -o 004_point_cloud.out -I../include/

Run the output file using the following command....
./004_point_cloud.out
*/

# include <stdio.h>
# include <unistd.h>
# include <sys/types.h>

# include "PAL.h"

int main(int argc, char** argv)
{
	int image_width = -1;
	int image_height = -1;

    //////PAL related code
    if(PAL::Init(image_width, image_height,-1) != PAL::SUCCESS)
        return 1; //Init failed
        

	////  *** NEW CODE ***
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

    PAL::Destroy();
	
	return 0;

}