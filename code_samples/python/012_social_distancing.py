#CODE SAMPLE # 012: People Tracking
#This code sample allows users to check if the distance between two people is within a limit or not.

import sys
import PAL_PYTHON
import cv2
import numpy as np
import math
from Xlib.display import Display

def GetUserInput(prompt):
    while True:
        try:
            value = float(input(prompt))
            return value
        except ValueError:
            print("Invalid input. Please enter a valid float value.")

def is_socially_distant(p1, p2, threshold):
    distance = math.sqrt((p1["x"] - p2["x"]) ** 2 + (p1["y"] - p2["y"]) ** 2 + (p1["z"] - p2["z"]) ** 2)
    if distance <= threshold:
        return False
    return True

def compute_distance_matrix(track_info, threshold_distance):
    num_persons = len(track_info)
    distant_data = [True] * num_persons

    for i in range(num_persons):
        for j in range(i+1, num_persons):
            socially_distant = is_socially_distant(track_info[i]["locations_3d"], track_info[j]["locations_3d"], threshold_distance/100)
            if not socially_distant:
                distant_data[i] = False
                distant_data[j] = False
    
    return distant_data

def main():
    #Distance threshold in cm
    #The Distance threshold should be kept within 1m to 2m range.
    threshold_distance = GetUserInput("Enter the distance threshold in cm, eg 100\n")

    if threshold_distance > 200:
        threshold_distance = 200.0
        print("threshold distance set above maximum range. Setting to 2m")
    elif threshold_distance < 100:
        threshold_distance = 100.0
        print("threshold distance set below minumum range. Setting to 1m")

    #Camera index is the video index assigned by the system to the camera. 
    #By default we set it to 5. Specify the index if the value has been changed.
    camera_index = 5    
    
    arg = len(sys.argv)
    if arg == 2:
        camera_index = int(sys.argv[1])

    #Connect to the PAL camera    
    res_init = PAL_PYTHON.InitP(camera_index)

    if res_init != PAL_PYTHON.SUCCESSP:
        print("Camera Init failed\n")
        return

    #Setting API Mode
    PAL_PYTHON.SetAPIModeP(PAL_PYTHON.TRACKINGP)

    loaded_prop = {}
    loaded_prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
    
    #Loading camera properties from a text file
    loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", loaded_prop)
    if ack_load == PAL_PYTHON.INVALID_PROPERTY_VALUEP: 
        PAL_PYTHON.DestroyP()
        return
    if ack_load != PAL_PYTHON.SUCCESSP:
        print("Error Loading settings! Loading default values.")

    PAL_PYTHON.SetDepthModeInTrackingP(PAL_PYTHON.DEPTH_3DLOCATION_ONP)

    tracking_mode = PAL_PYTHON.PEOPLE_DETECTIONP
    success = PAL_PYTHON.SetModeInTrackingP(tracking_mode)

    detection_threshold = 0.30
    class_id = -1 #use -1 to set for all classes
    PAL_PYTHON.SetDetectionModeThresholdP(detection_threshold, class_id)
    
    # Creating a window
    source_window = 'PAL Social Distancing'
    cv2.namedWindow(source_window, cv2.WINDOW_AUTOSIZE)

    print("\n\nPress ESC to close the window.")
    print("Press f/F to toggle filter rgb property.")
    print("Press v/V to toggle Vertical Flip property.")
    print("Press m/M to toggle Fast Depth property")
    print("Press q/Q & a/A to increase and decrease detection threshold respectively\n\n")

    key = ' '

    # ESC
    while key != 27:
        left, right, depth, trackingData, camera_changed =  PAL_PYTHON.GrabTrackingDataP()
        if camera_changed == True:
            break
        
        display = left
        
        num_of_persons = len(trackingData[PAL_PYTHON.OKP])

        DistantData = compute_distance_matrix(trackingData[PAL_PYTHON.OKP], threshold_distance)

        for i in range (0, num_of_persons):
            x1 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["x1"]
            y1 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["y1"]
            x2 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["x2"]
            y2 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["y2"]

            if(DistantData[i]):
                #Drawing GREEN box indicating the person is socially distant
                cv2.rectangle(display, (x1, y1, x2, y2), (0,255,0),2)
            else:
                #Drawing RED box indicating the person is not socially distant 
                cv2.rectangle(display, (x1, y1, x2, y2), (0,0,255),2)

        cv2.imshow(source_window, display)

        # Wait for 1ms
        key = cv2.waitKey(1) & 255

        #q
        if key == 113:
            detection_threshold += 0.1
            if(detection_threshold >1):
                detection_threshold = 1
                print("Max threshold (1.0) reached")

            PAL_PYTHON.SetDetectionModeThresholdP(detection_threshold, class_id)

        #a
        if key == 97:
            detection_threshold -= 0.1
            if(detection_threshold < 0.01):
                detection_threshold = 0.01
                print("Min threshold (0.01) reached")

            PAL_PYTHON.SetDetectionModeThresholdP(detection_threshold, class_id)
        
        #f
        if key == 102:            
            flag = PAL_PYTHON.FILTER_SPOTSP
            loaded_prop["filter_spots"] = not(bool(loaded_prop["filter_spots"]))
            loaded_prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)

        #v
        if key == 118:            
            flag = PAL_PYTHON.VERTICAL_FLIPP
            loaded_prop["vertical_flip"] = not(bool(loaded_prop["vertical_flip"]))
            loaded_prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)
        
        #m    
        if key == 109:
            flag = PAL_PYTHON.FDP
            loaded_prop["fd"] = not(bool(loaded_prop["fd"]))
            loaded_prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)

        # Destroying connections
    print("exiting the application\n")
    PAL_PYTHON.DestroyP()

    return

if __name__ == "__main__":
    main()
