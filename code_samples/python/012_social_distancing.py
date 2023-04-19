#CODE SAMPLE # 012: People Tracking
#This code sample allows users to check if the distance between two people is within a limit or not.

import sys
import PAL_PYTHON
import cv2
import numpy as np
import math
from Xlib.display import Display
import time

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

    # Initialising camera
    image_width = 0
    image_height = 0
    camera_index = 5

    width, height, res_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

    if res_init!= PAL_PYTHON.SUCCESSP:
        print("Camera Init failed\n")
        return

    PAL_PYTHON.SetAPIModeP(PAL_PYTHON.TRACKINGP)
    
    loaded_prop = {}
    prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
    
    loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", prop)
    if ack_load == PAL_PYTHON.INVALID_PROPERTY_VALUEP: 
        PAL_PYTHON.DestroyP()
        return
        
    if ack_load != PAL_PYTHON.SUCCESSP:
        print("Error Loading settings! Loading default values.")
        
    enableDepth = True
    PAL_PYTHON.SetDepthModeInTrackingP(PAL_PYTHON.DEPTH_3DLOCATION_ONP)

    tracking_mode = PAL_PYTHON.PEOPLE_DETECTIONP
    success = PAL_PYTHON.SetModeInTrackingP(tracking_mode)

    detection_threshold = 0.30
    class_id = -1 #use -1 to set for all classes
    PAL_PYTHON.SetDetectionModeThresholdP(detection_threshold, class_id)
    
    # Creating a window
    source_window = 'PAL Social Distancing'
    cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
    
    screen = Display().screen()
    sc_height = screen.height_in_pixels
    sc_width  = screen.width_in_pixels
    
    # Changing window size
    cv2.resizeWindow(source_window, sc_width-60, sc_height-60)

    threshold_distance = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0

    if threshold_distance > 200:
        threshold_distance = 200.0
        print("threshold distance set above maximum range. Setting to 2m")
    elif threshold_distance < 100:
        threshold_distance = 100.0
        print("threshold distance set below minumum range. Setting to 1m")
    
    key = ' '
    filter_spots = loaded_prop["filter_spots"]
    vertical_flip = loaded_prop["vertical_flip"]
    fd = loaded_prop["fd"]

    print("\n\nPress ESC to close the window.")
    print("Press f/F to toggle filter rgb property.")
    print("Press v/V to toggle Vertical Flip property.")
    print("Press m/M to toggle Fast Depth property")
    print("Press q/Q & a/A arrow key to increase and decrease detection threshold respectively\n\n")

    # ESC
    while key != 27:

        left, right, depth, trackingData, camera_changed =  PAL_PYTHON.GrabTrackingDataP()
        if camera_changed == True:
        	break
        display = left
        
        num_of_persons = len(trackingData[PAL_PYTHON.OKP])

        if(num_of_persons>=2):

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

        elif(num_of_persons==1):
            x1 = trackingData[PAL_PYTHON.OKP][0]["boxes"]["x1"]
            y1 = trackingData[PAL_PYTHON.OKP][0]["boxes"]["y1"]
            x2 = trackingData[PAL_PYTHON.OKP][0]["boxes"]["x2"]
            y2 = trackingData[PAL_PYTHON.OKP][0]["boxes"]["y2"]
            cv2.rectangle(display, (x1, y1, x2, y2), (0,255,0), 2)

        cv2.imshow(source_window, display)
        
        #print_track(trackingData)

            # Wait for 1ms
        key = cv2.waitKey(1) & 255

        #up-arrow key
        if key == 113:
            detection_threshold += 0.1
            if(detection_threshold >1):
                detection_threshold = 1
                print("Max threshold (1.0) reached")

            PAL_PYTHON.SetDetectionModeThresholdP(detection_threshold, class_id)

        #down-arrow key
        if key == 97:
            detection_threshold -= 0.1
            if(detection_threshold < 0.01):
                detection_threshold = 0.01
                print("Min threshold (0.01) reached")

            PAL_PYTHON.SetDetectionModeThresholdP(detection_threshold, class_id)
        
        #f
        if key == 102:            
            flag = PAL_PYTHON.FILTER_SPOTSP
            filter_spots = not(filter_spots)
            loaded_prop["filter_spots"] = filter_spots
            prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)
        
        #v    
        if key == 118:            
            flag = PAL_PYTHON.VERTICAL_FLIPP
            vertical_flip = not(vertical_flip)
            loaded_prop["vertical_flip"] = vertical_flip
            prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)
        
        #m    
        if key == 109:
            flag = PAL_PYTHON.FDP
            fd = not(fd)
            loaded_prop["fd"] = fd
            prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)


        # Destroying connections
    print("exiting the application\n")
    PAL_PYTHON.DestroyP()

    return

if __name__ == "__main__":
    main()





