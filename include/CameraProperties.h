# ifndef PAL_CAMERA_PROPERTIES_H
# define PAL_CAMERA_PROPERTIES_H

#include "StructEnumInfo.h"

namespace PAL
{
	enum CaptureType
	{
		DUMMY,
		CAMERA
	};

	enum PowerLineFrequency
	{
		_AUTO,
		_50HZ,
		_60HZ
	};

	enum Projection
	{
		EQUI_RECTANGULAR = 0,
		PERSPECTIVE = 1,
		SPHERICAL = 2,
	};

	enum HDR_Mode 
	{
		HDR_OFF = 0x01,
		HDR_AUTO = 0x02,
		HDR_MANUAL = 0x03,
	};

	enum Tracking_Quality
	{
		STANDARD = 0,
		MEDIUM = 1,
		HIGHEST = 2,
	};	

	/**
     * @brief Enumeration for depth options in tracking mode.
     */
    enum DepthInTracking
    {
        DEPTH_OFF = 0,            /**< Depth calculation is off. */
        DEPTH_ON = 1,             /**< Depth calculation is on. */
        DEPTH_3DLOCATION_ON = 2,   /**< 3D location calculation based on depth is on. */
    };

	enum CameraPropertyFlags
	{
		BRIGHTNESS = 0x1,
		CONTRAST = 0x2,
		SATURATION = 0x4,
		GAMMA = 0x8,
		GAIN = 0x10,
		WHITE_BAL_TEMP = 0x20,
		SHARPNESS = 0x40,
		EXPOSURE = 0x80,
		FOCUS = 0x100,
		HUE = 0x200,
		AUTO_WHITE_BAL = 0x400,
		AUTO_GAIN = 0x800,
		AUTO_FOCUS = 0x1000,
		POWER_LINE_FREQUENCY = 0x2000,
		VERTICAL_FLIP = 0x4000,
		FILTER_SPOTS = 0x8000,
		PROJECTION = 0x10000,
		CAMERA_HEIGHT = 0x20000,
		GROUND_DETECTION = 0x40000,
		YAW = 0x80000,
		PITCH = 0x100000,
		RANGE = 0x200000,
		STARTHFOV = 0x400000,
		HFOV_RANGE = 0x800000,
		STARTVFOV = 0x1000000,
		ENDVFOV = 0x2000000,
		DEPTH_SCALE = 0x4000000,
		POINTCLOUD_DENSITY = 0x8000000,
		IMAGE_STABILIZATION = 0x10000000,
		DEPTH_STABILIZATION = 0x20000000,
		COLOR_DEPTH = 0x40000000,
		RAW_DEPTH = 0x80000000,
		STEREO_IMAGE_STABILIZATION = 0x100000000,
		AUTO_EXPOSURE_METHOD = 0x200000000,
		HID_FRAME_RATE = 0x400000000,
		HID_DENOISE = 0x800000000,
		HID_QFACTOR = 0x1000000000,
		HID_IHDR_MODE = 0x2000000000, 
		HID_IHDR_VALUE = 0x4000000000,
		ODOA_DEPTHTEMPORAL = 0x8000000000,
		ODOA_DEPTHSENSITIVITY = 0x10000000000,
		CLOTHES_LINING_MAX_HEIGHT = 0x20000000000,
		FD = 0x40000000000,
		TRACKING_QUALITY = 0x80000000000,
		DEPTH_IN_TRACKING = 0x100000000000,
		ALL = 0x1FFFFFFFFFFF,
	};

	struct CameraPropertyValues
	{

		float MAX_CLOTHES_LINING_MAX_HEIGHT = 1000;
		float MIN_CLOTHES_LINING_MAX_HEIGHT = 0;
		float DEFAULT_CLOTHES_LINING_MAX_HEIGHT = 130;
		
		int MAX_DEPTH_TEMPORAL = 10;
		int MIN_DEPTH_TEMPORAL = 0;
		int DEFAULT_DEPTH_TEMPORAL = 0;

		int MAX_SENSITIVITY_OFFSET = 100;
		int MIN_SENSITIVITY_OFFSET = -100;
		int DEFAULT_SENSITIVITY_OFFSET = 0;

		int MAX_HID_FRAME_RATE = 120;
		int MIN_HID_FRAME_RATE = 1;
		int DEFAULT_HID_FRAME_RATE = 30;
		
		int MAX_HID_DENOISE = 15;
		int MIN_HID_DENOISE = 0;
		int DEFAULT_HID_DENOISE = 8;
		
		int MAX_HID_QFACTOR = 96;
		int MIN_HID_QFACTOR = 10;
		int DEFAULT_HID_QFACTOR = 96;
		
		int MAX_HID_IHDR_VALUE = 4;
		int MIN_HID_IHDR_VALUE = 1;
		int DEFAULT_HID_IHDR_VALUE = 1;
		
		HDR_Mode DEFAULT_HID_IHDR_MODE = HDR_OFF;
		
		int MAX_AUTO_EXPOSURE_METHOD = 2;
		int MIN_AUTO_EXPOSURE_METHOD = 0;
		int DEFAULT_AUTO_EXPOSURE_METHOD = 1;
		
		int MAX_IMAGE_STABILIZATION = 9;
		int MIN_IMAGE_STABILIZATION = 0;
		int DEFAULT_IMAGE_STABILIZATION = 3;
		
		int MAX_STEREO_IMAGE_STABILIZATION = 9;
		int MIN_STEREO_IMAGE_STABILIZATION = 0;
		int DEFAULT_STEREO_IMAGE_STABILIZATION = 0;
		
		int MAX_DEPTH_STABILIZATION = 6;
		int MIN_DEPTH_STABILIZATION = 0;
		int DEFAULT_DEPTH_STABILIZATION = 5;
		
		int MAX_DEPTH_SCALE = 30;
		int MIN_DEPTH_SCALE = 1;
		int DEFAULT_DEPTH_SCALE = 5;
		
		int MAX_POINT_CLOUD_DENSITY = 25;
		int MIN_POINT_CLOUD_DENSITY = 1;
		int DEFAULT_POINT_CLOUD_DENSITY = 9;
		
		int MAX_BRIGHTNESS = 15;
		int MIN_BRIGHTNESS = -15;
		int DEFAULT_BRIGHTNESS = -2;
		
		int MAX_CONTRAST = 30;
		int MIN_CONTRAST = 0;
		int DEFAULT_CONTRAST = 5;
		
		int MAX_SATURATION = 60;
		int MIN_SATURATION = 0;
		int DEFAULT_SATURATION = 60;

		int MAX_GAMMA = 500;
		int MIN_GAMMA = 40;
		int DEFAULT_GAMMA = 300;

		int MAX_GAIN = 100;
		int MIN_GAIN = 0;
		int DEFAULT_GAIN = 2;

		int MAX_WHITE_BAL_TEMP = 10000;
		int MIN_WHITE_BAL_TEMP = 1000;
		int DEFAULT_WHITE_BAL_TEMP = 4250;
		
		int MAX_SHARPNESS = 127;
		int MIN_SHARPNESS = 0;
		int DEFAULT_SHARPNESS = 0;
		
		int MAX_EXPOSURE = 1000;
		int MIN_EXPOSURE = 1;
		int DEFAULT_EXPOSURE = 50;

		int MAX_FOCUS = 255;
		int MIN_FOCUS = 0;
		int DEFAULT_FOCUS = 0;

		int MAX_HUE = 2000;
		int MIN_HUE = -2000;
		int DEFAULT_HUE = 0;
				
		bool DEFAULT_AUTO_WHITE_BAL = 1;
		bool DEFAULT_AUTO_GAIN = 0;
		bool DEFAULT_AUTO_FOCUS = 0;

		CaptureType DEFAULT_CAPTURE_TYPE = CaptureType::CAMERA;
		PowerLineFrequency DEFAULT_POWER_LINE_FREQUENCY = _AUTO;

		bool DEFAULT_VERTICAL_FLIP = false;
		bool DEFAULT_FILTER_SPOTS = true;

		Projection DEFAULT_PROJECTION = EQUI_RECTANGULAR;
		
		bool DEFAULT_GROUND_DETECTION = true;		
		
		int MAX_YAW = 359;
		int MIN_YAW = 0;
		int DEFAULT_YAW = 0;
		
		int MAX_PITCH = 45;
		int MIN_PITCH = -45;
		int DEFAULT_PITCH = 0;
		
		int MAX_RANGE = 1000;
		int MAX_MIN_RANGE = 50;		
		int MAX_START_HFOV = 359;
		int MAX_HFOV_RANGE = 360;
		int MAX_START_VFOV = 58;
		int MAX_END_VFOV = 58;
		float MAX_CAMERA_HEIGHT = 300;

		int MIN_RANGE = 51;	
		int MIN_MIN_RANGE = 0;
		int MIN_START_HFOV = 0;
		int MIN_HFOV_RANGE = 1;
		int MIN_START_VFOV = -58;
		int MIN_END_VFOV = -58;
		float MIN_CAMERA_HEIGHT = 0;

		int DEFAULT_MIN_RANGE = 50;	
		int DEFAULT_RANGE = 1000;
		int DEFAULT_START_HFOV = 0;
		int DEFAULT_HFOV_RANGE = 360;
		int DEFAULT_START_VFOV = 58;
		int DEFAULT_END_VFOV = -52;
		float DEFAULT_CAMERA_HEIGHT = 65;
	
		bool DEFAULT_RAW_DEPTH  = false;
		bool DEFAULT_COLOR_DEPTH  = true;
		
		bool DEFAULT_FD = true;

		Tracking_Quality DEFAULT_TRACKING_QUALITY = Tracking_Quality::STANDARD;

		DepthInTracking DEFAULT_DEPTH_IN_TRACKING = DepthInTracking::DEPTH_OFF;

		CameraPropertyValues();
	};
	
	struct CameraProperties
	{
		int brightness;
		int contrast;
		int saturation;
		int gamma;
		int gain;
		int white_bal_temp;
		int sharpness;
		int exposure;
		int hue;
		int focus;
		bool auto_white_bal;
		bool auto_gain;
		bool auto_focus;
		
		CaptureType capture_type;
		PowerLineFrequency power_line_frequency;

		bool  vertical_flip;
		bool  filter_spots;
		bool raw_depth;
		bool color_depth;

		//Projection type : equi-rectangular or perspective
		Projection projection;

		bool ground_detection;

		int yaw;
		int pitch;

		int range;
		int min_range;

		int start_hfov; 
		int hfov_range; 
		int start_vfov; 
		int end_vfov; 
		
		float camera_height;

		int depth_scale_factor;
		int point_cloud_density;

		int stereo_image_stabilization;
		int image_stabilization;
		int depth_stabilization;
		
		int auto_exposure_method;
		
		int hid_frame_rate;
		int hid_denoise;
		int hid_qfactor;
		int hid_ihdr_value;
		HDR_Mode hid_ihdr_mode;

		int depth_context_temporal;
		int sensitivity_offset;

		float clothes_lining_max_height;
		bool fd;

		Tracking_Quality tracking_quality;
		DepthInTracking depth_in_tracking;

		
		CameraProperties();	
	};
}//namespace

# endif //PAL_CAMERA_PROPERTIES_H

