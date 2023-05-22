# ifndef PAL_H
# define PAL_H

//This is the only file the end-user needs to include in the application
//All the functionality provided by the API is covered here.

# include <vector>

# include "DataExchange.h"
# include "CameraProperties.h" 

namespace PAL
{
		//Initializes the PAL API
		//returns SUCCESS/FAILURE
		PAL::Acknowledgement Init(int& panoramaWidth, int& panoramaHeight, std::vector<int> camera_indexes, void* arg=nullptr);


        PAL::Acknowledgement SetPathtoData(std::string path1, std::string path2="");

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
		PAL::Acknowledgement SetCameraProperties(PAL::CameraProperties* properties, unsigned long int *flags);


		//This function resets the camera properties.
		//If this function is called before the Init function, it would be ignored. Else, returns SUCCESS
		//If a pointer to PAL::CameraProperties is provided, the default values would be written into that location
		PAL::Acknowledgement SetDefaultCameraProperties(PAL::CameraProperties* properties = 0);
		

		//If a vector of Points is provided, those points would be saved with the mentioned fileName
		//If the vector is not provided, GetPointCloud function would be used internally
		PAL::Acknowledgement SavePointCloud(const char* fileName, std::vector<PAL::Point> *pc = 0);
		

		//Saves the current camera properties into the provided file name
		PAL::Acknowledgement SaveProperties(const char* fileName);


		//Loads the camera properties saved in the provided file
		//If data argument is provided, the properties in the file would be written into data
	    PAL::Acknowledgement LoadProperties(const char* fileName, PAL::CameraProperties* data = 0);

		
		//Destroys all the resources related to Camera communication	
		void Destroy();
		
		void UpdateZoomParams(int updated_Width, int updated_Height, int x_c, int y_c);
		void DisableModels(bool flag);

		//Use it with GPIO application on devices with low resources to improve performance
		void SyncronizeInputs(bool flag);
		
		void SetAppMode(int app);

		std::vector<PAL::Data::Stereo> GetStereoData();
		PAL::Data::Depth  GetDepthData();
		PAL::Data::People GetPeopleData(bool depth);
		PAL::Data::PointCloud GetPointCloudData();
		bool SavePointCloud(const char* fileName, cv::Mat pcMat);
		std::vector<PAL::Data::ODOA_Data> GrabRangeScanData();
		void SetCliffMaskFlag(bool flag);
		void PauseComputation(bool flag);
		void SetAPIMode(int mode);
		void SetRemapRGBMode(bool rgb);


		//Grabs the left, right panorama along with the tracking data.  \
		//If depth is enbled then it will also grab the depth panoramas.
		std::vector<PAL::Data::TrackingResults> GrabTrackingData();

		//Choose b/w various tracking modes like people following, object tracking etc
		int SetModeInTracking(int mode);

		//Set ID of  the tracked object you want to following
		void SetTrackID(int id);

		//Get ID of the object being followed
		int GetTrackID();

		//Returns a vector of available resolutions.
        std::vector<PAL::Resolution> GetAvailableResolutions();

        //Depth calculation could be turned on or off in tracking mode
        void SetDepthModeInTracking(int mode);

        //Set different threshold for each class. Same value applies for each class when the class_id = -1.
        //Valid when SetModeInTracking() is used to set PEOPLE_DETECTION or OBJECT_DETECTION mode.
        void SetDetectionModeThreshold(float threshold, int class_id=-1);

        //A Utility function to visualise the tracking information. It draws bounding boxes around 
        //the tracked object and displays their ID. It will also display depth or 3d location based on set modes.
        void drawTracksOnImage(cv::Mat &img, const PAL::Data::TrackingResults &data, int mode,
        bool ENABLEDEPTH=false, bool ENABLE3D=false);

        //This is a blocking call, waits till all the pending depth / disparity computations are finished and returns.
		//This should be used only when asynchronous is true in GrabFrames function arguments
		//void Synchronize();
	
	PAL::Acknowledgement ColorDepthPostProcessing(cv::Mat &depth);
}

# endif //PAL_H
