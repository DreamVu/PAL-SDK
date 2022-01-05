# ifndef PAL_H
# define PAL_H

//This is the only file the end-user needs to include in the application
//All the functionality provided by the API is covered here.

# include <vector>

# include "PAL_Image.h"
# include "PAL_CameraProperties.h"

//#define EnablePersionDetection

namespace PAL
{

	struct Point
	{
		float x, y, z;
		unsigned char r, g, b, a;

		Point() : x(0.0f), y(0.0f), z(0.0f), r(0), g(0), b(0), a(255)
		{

		}

		Point(float x1, float y1, float z1, unsigned char r1, unsigned char g1, unsigned char b1)
			: x(x1), y(y1), z(z1), r(r1), g(g1), b(b1), a(255)
		{

		}
	};
    
	struct BoundingBox
	{
		int x1, y1, x2, y2;

		BoundingBox() : x1(0), y1(0), x2(0), y2(0)
		{

		} 

		BoundingBox(int a, int b, int c, int d) 
		    : x1(a), y1(b), x2(c), y2(d)
		{

		}
	};
    

	//Initializes the PAL API
	//returns SUCCESS/FAILURE etc.
	PAL::Acknowledgement Init(int& panoramaWidth, int& panoramaHeight, int cameraIndex = -1);
	
	//Grabs the latest panoramas available
	//returns SUCCESS/FAILURE etc.
	//ARGUMENTS:
	//left			: Panorama - as seen by the left eye - packed in memory same as OpenCV CV_8UC3
	//right			: Panorama - as seen by the right eye - packed in memory same as OpenCV CV_8UC3
	//depth			: Panorama - as seen by the depth sensor - packed in memory same as OpenCV CV_32FC1
	//disparity		: disparity image, based on which the depth image is computed
	//normalize		: If true, the disparity would be normalized (CV_8UC1), if false unnormalized disparity (CV_16SC1)
	//asynchronous  : If true, the function returns immediately with the last computed values for disparity and depth.
	PAL::Acknowledgement GrabFrames(PAL::Image* left, 
									PAL::Image* right, 
									PAL::Image* depth = 0, 
									PAL::Image* disparity = 0, bool normalize = false,
									bool asynchronous = true);

	//This is a blocking call, waits till all the pending depth / disparity computations are finished and returns.
	//This should be used only when asynchronous is true in GrabFrames function arguments
	void Synchronize();


	//Writes the current camera properties into the provided memory location
	PAL::Acknowledgement GetCameraProperties(PAL::CameraProperties* properties);


	// SetCameraProperties
	// changes the camera properties like gamma, saturation etc.
	//
	// ARGUMENTS:
	// flags (read/write)		: Should point to a value formed by one/more combinations of CameraPropertyFlags
	// properties (readonly)	: Only those members are updated, who correspondings flags are set.
	//
	/* EXAMPLE:

		PAL::CameraProperties properties;
		properties.saturation = 2.0f;
		properties.gamma = 30000.0f;
		int flags = PAL::SATURATION | PAL::GAMMA;
		PAL_SDK::SetCameraProperties(&properties, &flags);
	*/
	// RETURNS:
	// 
	// returns SUCCESS/INVALID_PROPERTY_VALUE etc.
	// On successful return, flags should point to zero.
	// In case if an invalid properties are sent, 
	// the corresponding CameraPropertyFlags would be set to the int location pointed by flags
	// Refer API Doc for more information about this function
	PAL::Acknowledgement SetCameraProperties(PAL::CameraProperties* properties, unsigned int* flags = 0);


	//Returns a vector of available resolutions. 
	//While changing the resolution through SetCameraProperties...
	//users can use one of the available resolutions
	std::vector<PAL::Resolution> GetAvailableResolutions();


	//This function resets the camera properties.
	//If this function is called before the Init function, it would be ignored. Else, returns SUCCESS
	//If a pointer to PAL::CameraProperties is provided, the default values would be written into that location
	PAL::Acknowledgement SetDefaultCameraProperties(PAL::CameraProperties* properties = 0);
	

	//Analyzes the depth panorama and computes a vector of 3D points (with xyz and rgb information)
	PAL::Acknowledgement GetPointCloud(std::vector<PAL::Point> *pc, timeval *timestamp = 0,
		PAL::Image *left = 0, PAL::Image *right = 0, PAL::Image *depth = 0, PAL::Image *disparity = 0);

	//If a vector of Points is provided, those points would be saved with the mentioned fileName
	//If the vector is not provided, GetPointCloud function would be used internally
	PAL::Acknowledgement SavePointCloud(const char* fileName, std::vector<PAL::Point> *pc = 0);

	//Saves the current camera properties into the provided file name
	PAL::Acknowledgement SaveProperties(const char* fileName);

	//Loads the camera properties saved in the provided file
	//If data argument is provided, the properties in the file would be written into data
	PAL::Acknowledgement LoadProperties(const char* fileName, PAL::CameraProperties* data = 0);

    	//Initializes the Person Detection API from the provided path
	//returns SUCCESS/FAILURE etc.
	//ARGUMENTS:
	//threshold	: Confidence threshold of the detections ranging from 0 to 1 
    	PAL::Acknowledgement InitPersonDetection(float threshold = 0.3);

	//Grabs the latest person detection data available
	//returns SUCCESS/FAILURE etc.
	//ARGUMENTS:
	//left			: Panorama - as seen by the left eye - OpenCV CV_8UC3 type
	//depth			: Panorama - depth values in cm as seen by the depth sensor - OpenCV CV_32FC1 type
	//BoundingBoxes : writes bounding box values of the detections in the provided memory location
	//Loc3Ds		: writes 3D locations of the detections in the provided memory location
	//timestamp     : writes timestamp value of the data retrieved 
	PAL::Acknowledgement GetPeopleDetection(cv::Mat& rgb, cv::Mat& depth,    
		                            std::vector<PAL::BoundingBox> *BoundingBoxes, 
		                            timeval *timestamp = 0);

	//Sets height of camera from floor required for floor mapping
	//ARGUMENTS:
	//camera_height: Approximate Height of PAL camera from the floor in cm
	PAL::Acknowledgement SetFloorMappingParameters(int camera_height = 100);
	
	//Destroys all the resources related to Camera communication	
	void Destroy();
}

# endif //PAL_H
