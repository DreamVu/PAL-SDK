#ifndef PAL_H
#define PAL_H

/**
 * @file PAL.h
 * @brief This is the main header file for the PAL API.
 * All the functionality provided by the API is covered here.
 */

#include <vector>
#include "StructEnumInfo.h"
#include "CameraProperties.h"

namespace PAL
{
    /**
     * @brief Initializes the PAL API.
     * @param camera_indexes A vector containing the video indexes of cameras to be initialized.
     * @return PAL::Acknowledgement SUCCESS if initialization is successful, FAILURE otherwise.
     */
    PAL::Acknowledgement Init(std::vector<int> camera_indexes);

    /**
     * @brief Retrieves the current camera properties.
     * @param properties A pointer to a PAL::CameraProperties object where the properties will be written.
     * @return PAL::Acknowledgement SUCCESS if properties are successfully retrieved, FAILURE otherwise.
     */
    PAL::Acknowledgement GetCameraProperties(PAL::CameraProperties* properties);

    /**
     * @brief Changes the camera properties like gamma, saturation, etc.
     * @param properties A pointer to a PAL::CameraProperties object containing the updated properties.
     *                   Only those members whose corresponding flags are set are updated.
     * @param flags A pointer to an unsigned long int that should point to a value formed by one/more combinations of CameraPropertyFlags.
     * @return PAL::Acknowledgement SUCCESS if the properties are successfully set, INVALID_PROPERTY_VALUE, etc. on failure.
     *         On successful return, flags should point to zero.
     *         In case of invalid properties, the corresponding CameraPropertyFlags would be set to the int location pointed by flags.
     *         Refer to the API documentation for more information about this function.
     * @code
     * // EXAMPLE:
     * PAL::CameraProperties properties;
     * properties.saturation = 2.0f;
     * properties.gamma = 30000.0f;
     * unsigned long flags = PAL::SATURATION | PAL::GAMMA;
     * PAL::SetCameraProperties(&properties, &flags);
     * @endcode
     */
    PAL::Acknowledgement SetCameraProperties(PAL::CameraProperties* properties, unsigned long int* flags);

    /**
     * @brief Resets the camera properties to their default values.
     * @param properties A pointer to a PAL::CameraProperties object where the default values will be written (optional).
     *                   If not provided (nullptr), the default values will not be written.
     * @return PAL::Acknowledgement SUCCESS if the properties are reset successfully, FAILURE otherwise.
     *         If this function is called before the Init function, it will be ignored, and SUCCESS will be returned.
     */
    PAL::Acknowledgement SetDefaultCameraProperties(PAL::CameraProperties* properties = nullptr);

    /**
     * @brief Saves the current camera properties to a file.
     * @param fileName The name of the file where the properties will be saved.
     * @return PAL::Acknowledgement SUCCESS if the properties are saved successfully, FAILURE otherwise.
     */
    PAL::Acknowledgement SaveProperties(const char* fileName);

    /**
     * @brief Loads the camera properties saved in the provided file.
     * @param fileName The name of the file from which properties will be loaded.
     * @param data A pointer to a PAL::CameraProperties object where the loaded properties will be written (optional).
     *             If not provided (nullptr), the loaded properties will not be written.
     * @return PAL::Acknowledgement SUCCESS if the properties are loaded successfully, FAILURE otherwise.
     */
    PAL::Acknowledgement LoadProperties(const char* fileName, PAL::CameraProperties* properties = 0);

    /**
     * @brief Sets the API mode.
     * @param mode The mode to be set.
     */
    void SetAPIMode(int mode);

    /**
     * @brief Saves the point cloud of the current scene.
     * @param fileName The name of the file where the point cloud will be saved.
     * @param pcMat The point cloud data as a cv::Mat object.
     * @return true if the point cloud is saved successfully, false otherwise.
     *         Valid when SetAPIMode() is used to set POINT_CLOUD mode.
     */
    bool SavePointCloud(const char* fileName, cv::Mat pcMat);

    /**
     * @brief Grabs stereo mode data.
     * @return A vector containing PAL::Data::Stereo_Data objects representing stereo data.
     *         Valid when SetAPIMode() is used to set STEREO mode.
     */
    std::vector<PAL::Data::Stereo_Data> GrabStereoData();

    /**
     * @brief Grabs range scan mode data.
     * @return A vector containing PAL::Data::ODOA_Data objects representing range scan data.
     * *         Valid when SetAPIMode() is used to set DEPTH, RANGE_SCAN or POINT_CLOUD mode.
     */
    std::vector<PAL::Data::ODOA_Data> GrabRangeScanData();

    /**
     * @brief Grabs tracking mode data.
     * @return A vector containing PAL::Data::Tracking_Data objects representing tracking data.
     *         Valid when SetAPIMode() is used to set TRACKING mode.
     */
    std::vector<PAL::Data::Tracking_Data> GrabTrackingData();

    /**
     * @brief Sets the tracking mode.
     * @param mode The tracking mode to be set.
     * @return 0 if the operation is successful. If the current properties need to be modified to change the mode successfully, the function returns 1.
     */
    int SetModeInTracking(int mode);

    /**
     * @brief Sets different thresholds for each class in detection mode.
     * @param threshold The threshold value to be set.
     * @param class_id The class ID for which the threshold is set (optional, default is -1).
     *                 If class_id is -1, the same value applies for each class.
     *                 Valid when SetModeInTracking() is used to set PEOPLE_DETECTION or OBJECT_DETECTION mode.
     */
    void SetDetectionModeThreshold(float threshold, int class_id = -1);

    /**
     * @brief Sets the ID of the tracked object to follow.
     * @param id The ID of the object to be followed.
     *           Valid when SetModeInTracking() is used to set PEOPLE_FOLLOWING or OBJECT_FOLLOWING mode.
     */
    void SetTrackID(int id);

    /**
     * @brief Gets the ID of the object being followed.
     * @return The ID of the object being followed.
     *         Valid when SetModeInTracking() is used to set PEOPLE_FOLLOWING or OBJECT_FOLLOWING mode.
     */
    int GetTrackID();

    /**
     * @brief A utility function to visualize the tracking information.
     * @param img The input image on which to draw bounding boxes and display tracking information.
     * @param data The PAL::Data::Tracking_Data object containing tracking information.
     * @param mode The tracking mode to determine what information to display.
     * @param properties The PAL::CameraProperties to be used for visualizing the data(optional, default is nullptr).
     *                   If not provided (nullptr), the current properties will be used.
     */
    void drawTracksOnImage(cv::Mat& img, const PAL::Data::Tracking_Data& data, int mode,
                           PAL::CameraProperties* properties=nullptr);


     /**
     * @brief A utility function to visualize the depth information.
     * @param depth The depth data as a cv::Mat of CV_32FC1 format. It is converted to a coloured depth map of CV_8UC3 format.
     * @return SUCCESS if depth processing is successful, FAILURE otherwise.
     */
    PAL::Acknowledgement ColorDepthPostProcessing(cv::Mat& depth);

    /**
     * @brief Synchronizes inputs to improve performance on devices with low resources.
     * @param flag Set to true to enable input synchronization, false to disable it.
     */
    void SyncronizeInputs(bool flag);

    /**
     * @brief Destroys all the resources related to Camera communication.
     * @details This function should be called when you are done using the PAL API and want to release
     * all resources related to camera communication. It cleans up any allocated resources, closes connections,
     * and prepares the system for safe shutdown. Make sure to call this function before exiting the application
     * to avoid any resource leaks and ensure proper cleanup.
     */
    void Destroy();
}

# endif //PAL_H
