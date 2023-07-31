# ifndef PAL_STRUCT_ENUM_INFO_H
# define PAL_STRUCT_ENUM_INFO_H

# include <sys/time.h> 
# include <opencv2/opencv.hpp>

namespace PAL
{
	enum API_Mode
	{
		STEREO = 0x1,
		DEPTH = 0x2,
		RANGE_SCAN = 0x4,
		POINT_CLOUD = 0x8,
		TRACKING = 0X10,
		ALL_MODE = 0x1F,
	};

	enum Tracking_Mode
	{
		PEOPLE_TRACKING = 0X1,
		PEOPLE_FOLLOWING = 0X2,
		OBJECT_TRACKING = 0X4,
		OBJECT_FOLLOWING = 0X8,
		OBJECT_DETECTION = 0X10,
		PEOPLE_DETECTION = 0X20,
	};

	enum DepthInTracking
	{
		DEPTH_OFF = 0x1,
		DEPTH_ON = 0x2,
		DEPTH_3DLOCATION_ON = 0x4,
	};

	enum States
    {
        OK = 0,
        SEARCHING = 1,
        TERMINATED = 2,
    };

     struct BoundingBox
    {
        float x1, y1, x2, y2;

        BoundingBox() : x1(0), y1(0), x2(0), y2(0)
        {

        } 

        BoundingBox(float a, float b, float c, float d) 
            : x1(a), y1(b), x2(c), y2(d)
        {

        }
    };

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

    struct Loc3D
    {
        float x, y, z;
        Loc3D() : x(0.0f), y(0.0f), z(0.0f)
        {
        } 
        Loc3D(float x1, float y1, float z1) 
            : x(x1), y(y1), z(z1)
        {
        }
    };

    namespace Data
    {
        struct Common
        {
            timeval timestamp;
            int iterations;
            bool camera_changed = false;

            Common():iterations(0){}
        };

        struct TrackND
        {
            float t_is_activated;
            float t_track_id;
            float active;
            PAL::BoundingBox boxes; 

            float t_score;
            float t_label;
            Loc3D locations_3d;
        };

        struct Stereo_Data : Common
        {
            cv::Mat stereo_left;
            cv::Mat stereo_right;
        };

        struct ODOA_Data : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat ground;
            cv::Mat depth;
            cv::Mat scan;
            cv::Mat scan_overlay_left;
            cv::Mat point_cloud;
            cv::Mat raw_depth;
        };

        struct Tracking_Data : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat depth;
            std::vector<std::vector<PAL::Data::TrackND>> tracking_info;
        };
    }
}//namespace

# endif //PAL_STRUCT_ENUM_INFO_H