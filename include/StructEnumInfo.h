#ifndef PAL_STRUCT_ENUM_INFO_H
#define PAL_STRUCT_ENUM_INFO_H

#include <sys/time.h>
#include <opencv2/opencv.hpp>

namespace PAL
{
    /**
     * @file PAL_StructEnumInfo.h
     * @brief Contains enumerations, structures, and data types used in the PAL API.
     */

    /**
     * @brief Enumerates different acknowledgement states.
     */
    enum Acknowledgement
    {
        IGNORED, /**< The action was ignored. */
        SUCCESS, /**< The action was successful. */
        FAILURE, /**< The action failed. */
        INVALID_PROPERTY_VALUE, /**< The provided property value(s) is/are invalid. */
        ERROR_CAMERA_NOT_INITIALIZED /**< An error occurred due to camera not being initialized. 
                                        Ensure that the function PAL::Init() has been invoked. */
    };

    /**
     * @brief Enumeration for different API modes.
     */
    enum API_Mode
    {
        STEREO = 0x1,              /**< Stereo mode. */
        DEPTH = 0x2,               /**< Depth mode. */
        RANGE_SCAN = 0x4,          /**< Range scan mode. */
        POINT_CLOUD = 0x8,         /**< Point cloud mode. */
        TRACKING = 0X10,           /**< Tracking mode. */
        ALL_MODE = 0x1F            /**< All modes (combination of all modes). */
    };

    /**
     * @brief Enumeration for different tracking modes.
     */
    enum Tracking_Mode
    {
        PEOPLE_TRACKING = 0X1,     /**< People tracking mode. */
        PEOPLE_FOLLOWING = 0X2,    /**< People following mode. */
        OBJECT_TRACKING = 0X4,     /**< Object tracking mode. */
        OBJECT_FOLLOWING = 0X8,    /**< Object following mode. */
        OBJECT_DETECTION = 0X10,   /**< Object detection mode. */
        PEOPLE_DETECTION = 0X20    /**< People detection mode. */
    };

    /**
     * @brief Enumeration representing the tracking states for different track IDs.
     * @details The tracking state indicates the status of track IDs in the current frame.
     */
    enum States
    {
        OK = 0,         /**< The track IDs in this state are currently being actively tracked. */
        SEARCHING = 1,  /**< The track IDs in this state couldn't be found in the current frame but are still being searched for. */
        TERMINATED = 2  /**< The track IDs in this state have been lost and are no longer being searched for. */
    };


    /**
     * @brief Structure representing a bounding box with coordinates.
     */
    struct BoundingBox
    {
        float x1; /**< x coordinate of the Top-Left corner of the bounding box. */
        float y1; /**< y coordinate of the Top-Left corner of the bounding box. */
        float x2; /**< Width of the bounding box. */
        float y2; /**< Height of the bounding box. */

        BoundingBox() : x1(0), y1(0), x2(0), y2(0) {}

        BoundingBox(float a, float b, float c, float d)
            : x1(a), y1(b), x2(c), y2(d) {}
    };

    /**
     * @brief Structure representing a 3D point with RGBA color.
     */
    struct Point
    {
        float x, y, z;
        unsigned char r, g, b, a;

        Point() : x(0.0f), y(0.0f), z(0.0f), r(0), g(0), b(0), a(255) {}

        Point(float x1, float y1, float z1, unsigned char r1, unsigned char g1, unsigned char b1)
            : x(x1), y(y1), z(z1), r(r1), g(g1), b(b1), a(255) {}
    };

    /**
     * @brief Structure representing a 3D location.
     */
    struct Loc3D
    {
        float x, y, z;
        Loc3D() : x(0.0f), y(0.0f), z(0.0f) {}
        Loc3D(float x1, float y1, float z1) : x(x1), y(y1), z(z1) {}
    };

    namespace Data
    {
        /**
         * @brief Common data shared among different PAL data structures.
         */
        struct Common
        {
            timeval timestamp;         /**< Time timestamp of the data. */
            int iterations;            /**< Iterations count. */
            bool camera_changed;       /**< Flag indicating if the camera changed or disconnected. */

            Common() : iterations(0), camera_changed(false) {}
        };

        /**
         * @brief Structure representing tracking information for individual objects.
         */
        struct TrackND
        {
            float t_is_activated;      /**< Flag indicating if the track is activated. */
            float t_track_id;          /**< ID of the tracked object. */
            float active;              /**< Flag indicating if the object is active. */
            PAL::BoundingBox boxes;    /**< Bounding box of the object. */

            float t_score;             /**< Detection confidense score of the object. */
            float t_label;             /**< Class Label of the tracked object. */
            Loc3D locations_3d;        /**< 3D locations of the object. */
        };

        /**
         * @brief Structure representing stereo data.
         */
        struct Stereo_Data : Common
        {
            cv::Mat stereo_left;       /**< Left stereo image. */
            cv::Mat stereo_right;      /**< Right stereo image. */
        };

        /**
         * @brief Structure representing ODOA data.
         */
        struct ODOA_Data : Common
        {
            cv::Mat left;               /**< Left image. */
            cv::Mat right;              /**< Right image. */
            cv::Mat depth;              /**< Depth map. */
            cv::Mat raw_depth;          /**< Raw depth map. */
            cv::Mat scan;               /**< Scan data. */
            cv::Mat scan_overlay_left;  /**< Overlay of scan on the left image. */
            cv::Mat ground;             /**< Ground information. */
            cv::Mat point_cloud;        /**< Point cloud data. */
        };

        /**
         * @brief Structure representing tracking data.
         */
        struct Tracking_Data : Common
        {
            cv::Mat left;               /**< Left image. */
            cv::Mat right;              /**< Right image. */
            cv::Mat depth;              /**< Depth map. */
            std::vector<std::vector<PAL::Data::TrackND>> tracking_info; /**< Tracking information. */
        };
    } // namespace Data
} // namespace PAL

#endif // PAL_STRUCT_ENUM_INFO_H
