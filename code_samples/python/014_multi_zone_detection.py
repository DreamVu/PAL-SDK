#CODE SAMPLE # 014: Multi Zone Detection
#This code will grab the left panorama with person tracking data and check if they are in the user provided distinctive detection zones

import sys
import PAL_PYTHON
import cv2
import numpy as np
import math
from Xlib.display import Display
import time

def precision_string(num, precision=1):
    num_string = str(num)
    return num_string[0:num_string.find(".")+1+precision]

def zoneDetection(img, trackingData, thresh):
    classes = ["person","bicycle","car","motorcycle","airplane",
    "bus","train","truck","boat","traffic light","fire hydrant",
    "stop sign","parking meter","bench","bird","cat","dog","horse",
    "sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee",
    "skis","snowboard","sports ball","kite","baseball bat",
    "baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot",
    "hot dog","pizza","donut","cake","chair","couch","potted plant",
    "bed","dining table","toilet","tv","laptop","mouse","remote",
    "keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"]

    only_detection = False

    ENABLEDEPTH = True
    ENABLE3D = True

    
    no_of_persons = len(trackingData[PAL_PYTHON.OKP])

    print("-----------------------------------------------")
    
    for i in range (0, no_of_persons):        
        x1 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["x1"]
        y1 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["y1"]
        x2 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["x2"]
        y2 = trackingData[PAL_PYTHON.OKP][i]["boxes"]["y2"]

        x3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["x"]
        y3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["y"]
        z3D = trackingData[PAL_PYTHON.OKP][i]["locations_3d"]["z"]

        depth_value = math.sqrt(x3D*x3D + y3D*y3D)

        fontface = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.4
        thickness = 1
        baseline = 0
        height_mul = 3
        text2_height = 0
        text2_width = 0

        if only_detection:
            label1 = "Class= " + classes[ int(round(trackingData[PAL_PYTHON.OKP][i]["t_label"])) ]
        else:
            label1 = "ID=" + str(int(trackingData[PAL_PYTHON.OKP][i]["t_track_id"])) + ", "+classes[ int(round(trackingData[PAL_PYTHON.OKP][i]["t_label"])) ]

        if ENABLEDEPTH:
            if ENABLE3D:
                label1 += ", Depth=" + precision_string(depth_value,1) + "m"
                label2 = "x:" + precision_string(x3D,1) + "m, y:" + precision_string(y3D,1) + "m, z:" + precision_string(z3D,1) + "m"
            else:
                label2 = "Depth=" + precision_string(depth_value,1) + "m"
            
            text2, _ = cv2.getTextSize(label2, fontface, scale, thickness)
            text2_width, text2_height = text2
            baseline += thickness
        
        text1, _ = cv2.getTextSize(label1, fontface, scale, thickness)
        text1_width, text1_height = text1
        baseline += thickness

        box_height = text1_height + text2_height + baseline + 3
        box_width = max(text1_width, text2_width) + 2

        cv2.rectangle(img, (x1, y1-box_height-1, box_width, box_height), (255,255,255), cv2.FILLED)        

        number_of_zones = len(thresh)
        beyond_all_zones = True

        for j in range(0,number_of_zones):
            if(depth_value < thresh[j]):
                print("ID " + str(int(trackingData[PAL_PYTHON.OKP][i]["t_track_id"])) + " belongs to zone " + str(j+1))
                normalised_color = float(j)/float(number_of_zones);
                green = int(255.0*normalised_color);
                red = int(255.0*(1.0-normalised_color));
                colors = (0, green, red)
                beyond_all_zones = False
                break

        if beyond_all_zones == True:
            print("ID " + str(int(trackingData[PAL_PYTHON.OKP][i]["t_track_id"])) + " belongs beyond zone " + str(number_of_zones))
            colors = (0, 255, 0)

        cv2.putText(img, label1, (x1, (y1-text2_height-baseline-2)), fontface, scale, colors, thickness, cv2.LINE_AA)
        if ENABLEDEPTH:
            cv2.putText(img, label2, (x1, int(y1-baseline/2 -1)), fontface, scale, colors, thickness, cv2.LINE_AA)

        cv2.rectangle(img, (x1, y1, x2, y2), colors, 2)
    
    tmp_string = "num: " + str(no_of_persons)
    cv2.putText(img, tmp_string, (0, 30), 0, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

    print("-----------------------------------------------")
    print("")


def print_track(results):
    print(results[PAL_PYTHON.OKP])
    for i in range(0,len(results[PAL_PYTHON.OKP])):
        print(" in OK TrackID: ", results[PAL_PYTHON.OKP][i]["t_track_id"])
        print(" in OK Activated: ", results[PAL_PYTHON.OKP][i]["t_is_activated"])
        print(" in OK Activate: ", results[PAL_PYTHON.OKP][i]["active"]) 
        print(" in OK x: ", results[PAL_PYTHON.OKP][i]["locations_3d"]["x"])
        print(" in OK y: ", results[PAL_PYTHON.OKP][i]["locations_3d"]["y"]) 
        print(" in OK z: ", results[PAL_PYTHON.OKP][i]["locations_3d"]["z"])
        print(" in OK Score  : ", results[PAL_PYTHON.OKP][i]["t_score"])
        print(" in OK Label  : ", results[PAL_PYTHON.OKP][i]["t_label"])
        print(" in OK boxes x1  : ", results[PAL_PYTHON.OKP][i]["boxes"]["x1"])
        print(" in OK boxes y1  : ", results[PAL_PYTHON.OKP][i]["boxes"]["y1"])
        print(" in OK boxes x2  : ", results[PAL_PYTHON.OKP][i]["boxes"]["x2"])
        print(" in OK boxes y2  : ", results[PAL_PYTHON.OKP][i]["boxes"]["y2"])
   
    if len(results)>1:
        for i in range(0,len(results[PAL_PYTHON.SEARCHINGP])):
            print(" in SEARCHING TrackID: ", results[PAL_PYTHON.SEARCHINGP][i]["t_track_id"])
            print(" in SEARCHING Activated: ", results[PAL_PYTHON.SEARCHINGP][i]["t_is_activated"])
            print(" in SEARCHING Activate: ", results[PAL_PYTHON.SEARCHINGP][i]["active"])
            print(" in SEARCHING x: ", results[PAL_PYTHON.SEARCHINGP][i]["locations_3d"]["x"])
            print(" in SEARCHING y: ", results[PAL_PYTHON.SEARCHINGP][i]["locations_3d"]["y"])
            print(" in SEARCHING z: ", results[PAL_PYTHON.SEARCHINGP][i]["locations_3d"]["z"])
            print(" in SEARCHING Score  : ", results[PAL_PYTHON.SEARCHINGP][i]["t_score"])
            print(" in SEARCHING Label  : ", results[PAL_PYTHON.SEARCHINGP][i]["t_label"])
            print(" in SEARCHING boxes x1  : ", results[PAL_PYTHON.SEARCHINGP][i]["boxes"]["x1"])
            print(" in SEARCHING boxes y1  : ", results[PAL_PYTHON.SEARCHINGP][i]["boxes"]["y1"])
            print(" in SEARCHING boxes x2  : ", results[PAL_PYTHON.SEARCHINGP][i]["boxes"]["x2"])
            print(" in SEARCHING boxes y2  : ", results[PAL_PYTHON.SEARCHINGP][i]["boxes"]["y2"])

        print("\n")
        for i in range(0,len(results[PAL_PYTHON.TERMINATEDP])):
            print(" in TERMINATED TrackID: ", results[PAL_PYTHON.TERMINATEDP][i]["t_track_id"])
            print(" in TERMINATED Activated: ", results[PAL_PYTHON.TERMINATEDP][i]["t_is_activated"])
            print(" in TERMINATED Activate: ", results[PAL_PYTHON.TERMINATEDP][i]["active"])
            print(" in TERMINATED x: ", results[PAL_PYTHON.TERMINATEDP][i]["locations_3d"]["x"])
            print(" in TERMINATED y: ", results[PAL_PYTHON.TERMINATEDP][i]["locations_3d"]["y"])
            print(" in TERMINATED z: ", results[PAL_PYTHON.TERMINATEDP][i]["locations_3d"]["z"])
            print(" in TERMINATED Score  : ", results[PAL_PYTHON.TERMINATEDP][i]["t_score"])
            print(" in TERMINATED Label  : ", results[PAL_PYTHON.TERMINATEDP][i]["t_label"])
            print(" in TERMINATED boxes x1  : ", results[PAL_PYTHON.TERMINATEDP][i]["boxes"]["x1"])
            print(" in TERMINATED boxes y1  : ", results[PAL_PYTHON.TERMINATEDP][i]["boxes"]["y1"])
            print(" in TERMINATED boxes x2  : ", results[PAL_PYTHON.TERMINATEDP][i]["boxes"]["x2"])
            print(" in TERMINATED boxes y2  : ", results[PAL_PYTHON.TERMINATEDP][i]["boxes"]["y2"])

        print("\n")

    print("\n")

def main():

    # Initialising camera
    image_width = 0
    image_height = 0
    camera_index = 5
    arg = len(sys.argv)

    if (arg < 2) or (int(sys.argv[1]) != (arg - 2)):
        print("Wrong format for arguments")
        print("Expected format:")
        print("python 014_multi_zone_detection.py <number of zones> <distance 1> <distance 2> .. <distance for last zone>")
        return

    number_of_zones = int(sys.argv[1])

    distances = []

    for i in range(0,number_of_zones):
        distances.append((float(sys.argv[i+2])/100.0))

    #distances.sort()

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
    
    PAL_PYTHON.SetDepthModeInTrackingP(PAL_PYTHON.DEPTH_3DLOCATION_ONP)

    tracking_mode = PAL_PYTHON.PEOPLE_TRACKINGP
    success = PAL_PYTHON.SetModeInTrackingP(tracking_mode)
    
    # Creating a window
    source_window = 'PAL Multi Zone Detection'
    cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
    
    screen = Display().screen()
    sc_height = screen.height_in_pixels
    sc_width  = screen.width_in_pixels
    
    # Changing window size
    cv2.resizeWindow(source_window, sc_width-60, sc_height-60)
    
    key = ' '
    filter_spots = loaded_prop["filter_spots"]
    vertical_flip = loaded_prop["vertical_flip"]
    fd = loaded_prop["fd"]

    print("\n\nPress ESC to close the window.")
    print("Press f/F to toggle filter rgb property.")
    print("Press v/V to toggle Vertical Flip property.")
    print("Press m/M to toggle Fast Depth property\n\n")

    # ESC
    while key != 27:

        left, right, depth, trackingData, camera_changed =  PAL_PYTHON.GrabTrackingDataP()
        if camera_changed == True:
            break
        display = left
        
        zoneDetection(display, trackingData, distances)
        
        cv2.imshow(source_window, display)

        # Wait for 1ms
        key = cv2.waitKey(1) & 255
        
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





