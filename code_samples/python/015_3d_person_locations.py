#CODE SAMPLE # 015: 3D Location
#This code will grab the left panorama with person detection data and overlay their 3D location

import sys
import PAL_PYTHON
import cv2
import numpy as np
import math
from Xlib.display import Display
import time

def setLabel(img, label, org, clr):
    fontface = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.4
    thickness = 1
    baseline = 0

    (text_width, text_height), baseline = cv2.getTextSize(label, fontface, scale, thickness)

    cv2.rectangle(img, (int(org[0]+0), int(org[1]+baseline)), (int(org[0]+text_width), int(org[1]-text_height)), (0,0,0), cv2.FILLED)
    cv2.putText(img, label, org, fontface, scale, clr, thickness, 4)

def draw3DLocation(img, trackingData):
    no_of_persons = len(trackingData[PAL_PYTHON.OKP])
    
    for i in range (0, no_of_persons):
        x1 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["x1"]
        y1 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["y1"]
        x2 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["x2"]
        y2 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["y2"]

        x3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["x"]
        y3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["y"]
        z3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["z"]

        depth_value = math.sqrt(x3D*x3D + y3D*y3D)

        text = str("x:{:.1f}m y:{:.1f}m z:{:.1f}m".format(round(x3D,1), round(y3D,1), round(z3D,1)))

        if(depth_value > 3):
            color = (0, 255, 0)
        else:
            color = (0,0,255)

        cv2.circle(img, (int(x1+x2/2), int(y1+y2/4)), 5, color, -1)
        setLabel(img, text, (int(x1+x2/2), int(y1+y2/4-10)), color)

def main():
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
    source_window = 'PAL 3D Location'
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
        draw3DLocation(display, trackingData)

        cv2.imshow(source_window, display)

        # Wait for 1ms
        key = cv2.waitKey(1) & 255

        #q key
        if key == 113:
            detection_threshold += 0.1
            if(detection_threshold >1):
                detection_threshold = 1
                print("Max threshold (1.0) reached")

            PAL_PYTHON.SetDetectionModeThresholdP(detection_threshold, class_id)

        #a key
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
