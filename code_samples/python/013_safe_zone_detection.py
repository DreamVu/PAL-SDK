#CODE SAMPLE # 013: Safe Zone Detection
#This code sample allows users to run Safe Zone Detection.

import sys
import PAL_PYTHON
import cv2
import numpy as np
import math
from Xlib.display import Display
import time

def setLabel(input_image, label, org, clr):
    fontface = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.8
    thickness = 2
    baseline = 0

    text_size, _ = cv2.getTextSize(label, fontface, scale, thickness)
    cv2.rectangle(input_image, (org[0] + 0, org[1] + baseline), (org[0] + text_size[0], org[1] - text_size[1]), (0, 0, 0), cv2.FILLED)
    cv2.putText(input_image, label, org, fontface, scale, clr, thickness, cv2.LINE_AA)

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
    source_window = 'PAL Safe Zone Detection'
    cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
    
    screen = Display().screen()
    sc_height = screen.height_in_pixels
    sc_width  = screen.width_in_pixels
    
    # Changing window size
    cv2.resizeWindow(source_window, sc_width-60, sc_height-60)

    safe_distance = float(sys.argv[1]) if len(sys.argv) > 1 else 100.0
    
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

        for i in range (0, num_of_persons):
            x1 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["x1"]
            y1 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["y1"]
            x2 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["x2"]
            y2 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["y2"]

            x3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["x"]
            y3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["y"]
            z3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["z"]

            depth_value = math.sqrt(x3D*x3D + y3D*y3D)
            text = "Depth:{:.2f}m".format(depth_value)

            safe = depth_value*100 > safe_distance

            if(safe):
                #Drawing GREEN box indicating the person is socially distant
                cv2.rectangle(display, (x1, y1, x2, y2), (0,255,0),2)
            else:
                #Drawing RED box indicating the person is not socially distant 
                cv2.rectangle(display, (x1, y1, x2, y2), (0,0,255),2)

            setLabel(display, text, (x1,y1), (0,0,255))

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





