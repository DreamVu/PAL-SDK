# ifndef PAL_UNIFIED_DATA_EXCHANGE_H
# define PAL_UNIFIED_DATA_EXCHANGE_H

# include <sys/time.h> 
# include <opencv2/opencv.hpp>
namespace PAL
{    
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

     enum class Mode
    {
        IDLE,
        STEREO,
        DEPTH,
        PEOPLE_DETECTION,
        PEOPLE_DETECTION_WITH_DEPTH,
        // POINT_CLOUD_3D,       // rename later ---> python thread using it
        POINT_CLOUD,
        REFINED_LEFT,        //will remove later
        LASER_SCAN,
        SGBM
    };  
    
    namespace Data
    {
        struct Common
        {
            timeval timestamp;
            int iterations;

            Common():iterations(0){}
        };

        struct Camera : Common
        {
            cv::Mat cfm;
        };

        struct Stereo : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat refined_left;
            cv::Mat corrected_panorama;
            cv::Mat corrected_left_panorama;
            cv::Mat corrected_right_panorama;            
            cv::Mat corrected_panorama_de;
            cv::Mat mask;	
            cv::Mat mask_de;
            cv::Mat flow;
            
            cv::Mat stereo_left;
            cv::Mat stereo_right;
        };
        
        
        struct Cliff : Stereo
        {
            cv::Mat cliff;
        };
        
        struct CliffOutput : Common
        {
        	cv::Mat cliffmask;
        	cv::Mat col_sum;
        	cv::Mat opt_floor;
        	cv::Mat opt_wall;
        
        };
        
        struct Depth : Stereo
        {
            cv::Mat depth;
        };        
        
        struct People : Common
        {
            // std::vector<cv::Rect> bounding_boxes;
            std::vector<PAL::BoundingBox> bounding_boxes;
            cv::Mat left;
            cv::Mat depth;
        };    

        struct Refined : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat refined_left;
        };    
        
        struct FloorMask : Common
        {
            cv::Mat left;
            cv::Mat right;
            cv::Mat depth;
            cv::Mat floor_mask;
            cv::Mat display_rgb;
        };          
        
        struct FloorFocalMap : Common
        {
            cv::Mat left;
            cv::Mat depth;
            cv::Mat focal_map;
        };      
        
        struct PointCloud : Common
        {
            cv::Mat left;
            cv::Mat depth;
            cv::Mat point_cloud;
        };
        
        struct StereoDepthData : Common
	    {
	        cv::Mat left;
            cv::Mat right;
	        cv::Mat disparity; 
	        cv::Mat depth;
	        cv::Mat mask;
	    };
	
	    struct ODOA_Data : CliffOutput
	    {
	        cv::Mat left;
	        cv::Mat right;
            cv::Mat de_out; 
	        cv::Mat depth;
	        cv::Mat distance;
	        cv::Mat scan;
	        cv::Mat marked_left;
	        cv::Mat corrected_panorama;
	        cv::Mat mask;
	        cv::Mat mask_de;
	        cv::Mat flow;
	        cv::Mat point_cloud;
	        
	        cv::Mat stereo_left;
	        cv::Mat stereo_right;
	        cv::Mat fused_depth;
	    };
    }

}


# endif //PAL_UNIFIED_DATA_EXCHANGE_H
