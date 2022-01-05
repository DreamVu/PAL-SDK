# ifndef PAL_CAMERA_PROPERTIES_H
# define PAL_CAMERA_PROPERTIES_H

namespace PAL
{

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
		AUTO_WHITE_BAL = 0x100,
		AUTO_EXPOSURE = 0x200,
		RESOLUTION = 0x400,
		COLOR_SPACE = 0x800,
		POWER_LINE_FREQUENCY = 0x1000,
		VERTICAL_FLIP = 0x2000,
		FILTER_DISPARITY = 0x4000,
		FILTER_SPOTS = 0x8000,
        	FOV = 0x10000,
		PROJECTION = 0x20000,
		DISPARITY_COMPUTATION = 0x40000,
		ALL = 0x7FFFF,
	};

	struct Resolution
	{
		int width;
		int height;
	};


	namespace AvailableResolutions
	{
		const Resolution _5290x1819 = { .width = 5290,.height = 1819 };
		const Resolution _1744x600 = { .width = 1744,.height = 600 };
		const Resolution _1322x454 = { .width = 1322,.height = 454 };
		const Resolution _660x227 = { .width = 660,.height = 227 };
	}

	enum Acknowledgement
	{
		IGNORED, 
		SUCCESS, 
		FAILURE, 
		INVALID_PROPERTY_VALUE, 
		ERROR_CAMERA_NOT_INITIALIZED
	};

	enum ColorSpace
	{
		RGB,
		YUV444
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
	};

	enum DisparityComputation
	{
		FAST = 0,
		HIGH_QUALITY_A = 1,
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
		bool  auto_white_bal;
		bool  auto_exposure;

		Resolution resolution;
		ColorSpace color_space;
		PowerLineFrequency power_line_frequency;

			
		bool  vertical_flip;
		bool  filter_disparity;
		bool  filter_spots;
        
		//horizontal FOV in degrees
		int   fov_start; 
		int   fov_end;
 
		//Projection type : equi-rectangular or perspective
		Projection projection;

		//Should this be fast but of lower quality?
		//or of high quality but slow
		DisparityComputation computation;
        
       		static const int MAX_BRIGHTNESS = 15;
		static const int MIN_BRIGHTNESS = -15;
		static const int DEFAULT_BRIGHTNESS = 0;
		
		static const int MAX_CONTRAST = 30;
		static const int MIN_CONTRAST = 0;
		static const int DEFAULT_CONTRAST = 15;
		
		static const int MAX_SATURATION = 60;
		static const int MIN_SATURATION = 0;
		static const int DEFAULT_SATURATION = 32;

		static const int MAX_GAMMA = 500;
		static const int MIN_GAMMA = 40;
		static const int DEFAULT_GAMMA = 220;

		static const int MAX_GAIN = 100;
		static const int MIN_GAIN = 0;
		static const int DEFAULT_GAIN = 0;

		static const int MAX_WHITE_BAL_TEMP = 10000;
		static const int MIN_WHITE_BAL_TEMP = 1000;
		static const int DEFAULT_WHITE_BAL_TEMP = 5000;
		
		static const int MAX_SHARPNESS = 127;
		static const int MIN_SHARPNESS = 0;
		static const int DEFAULT_SHARPNESS = 0;
		
		static const int MAX_EXPOSURE = 10000;
		static const int MIN_EXPOSURE = 1;
		static const int DEFAULT_EXPOSURE = 312;
				
		static const bool DEFAULT_AUTO_WHITE_BAL = 1;
		static const bool DEFAULT_AUTO_EXPOSURE = 0;

		static const Resolution DEFAULT_RESOLUTION;
		static const ColorSpace DEFAULT_COLOR_SPACE = RGB;
		static const PowerLineFrequency DEFAULT_POWER_LINE_FREQUENCY = _AUTO;

		static const bool DEFAULT_VERTICAL_FLIP = false;
		static const bool DEFAULT_FILTER_DISPARITY = true;
		static const bool DEFAULT_FILTER_SPOTS = true;
        
		static const int DEFAULT_FOV_START = 0;
		static const int DEFAULT_FOV_END = 360;

		static const Projection DEFAULT_PROJECTION = PERSPECTIVE;
		static const DisparityComputation DEFAULT_COMPUTATION = HIGH_QUALITY_A;

		CameraProperties() :
			brightness           (DEFAULT_BRIGHTNESS),
			contrast  	     (DEFAULT_CONTRAST), 
			saturation           (DEFAULT_SATURATION),
			gamma                (DEFAULT_GAMMA),
			gain                 (DEFAULT_GAIN),
			white_bal_temp       (DEFAULT_WHITE_BAL_TEMP),
			sharpness            (DEFAULT_SHARPNESS),
			exposure 	     (DEFAULT_EXPOSURE),
			auto_white_bal       (DEFAULT_AUTO_WHITE_BAL),
			auto_exposure        (DEFAULT_AUTO_EXPOSURE),
			resolution           (DEFAULT_RESOLUTION),
			color_space          (DEFAULT_COLOR_SPACE),
			power_line_frequency (DEFAULT_POWER_LINE_FREQUENCY),
			vertical_flip        (DEFAULT_VERTICAL_FLIP),
			filter_disparity     (DEFAULT_FILTER_DISPARITY),
			filter_spots 	     (DEFAULT_FILTER_SPOTS),
			fov_start            (DEFAULT_FOV_START),
			fov_end              (DEFAULT_FOV_END),
			projection           (DEFAULT_PROJECTION),
			computation          (DEFAULT_COMPUTATION)
		{
		}
	};

}//namespace

# endif //PAL_CAMERA_PROPERTIES_H

