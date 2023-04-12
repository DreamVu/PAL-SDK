#CODE SAMPLE # 008: People Following
#This code sample will center on the person being tracked and follow him

import sys
import PAL_PYTHON
import cv2
import numpy as np
import math
from Xlib.display import Display
import time
import subprocess

def is_number(s):
    return s.isnumeric()

def get_color(idx):
    idx += 3
    return (37 * idx % 255, 17 * idx % 255, 29 * idx % 255)

def precision_string(num, precision=1):
    num_string = str(num)
    return num_string[0:num_string.find(".")+1+precision]

def drawOnImage(img, trackingData, mode, ENABLEDEPTH=False, ENABLE3D=False):
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

    if (mode == PAL_PYTHON.OBJECT_DETECTIONP) or (mode == PAL_PYTHON.PEOPLE_DETECTIONP):
        only_detection = True
    else:
        only_detection = False

    if ENABLEDEPTH == False:
        ENABLE3D = False

    
    no_of_persons = len(trackingData[PAL_PYTHON.OKP])

    for i in range (0, no_of_persons):
        colors = get_color(int(trackingData[PAL_PYTHON.OKP][i]["t_track_id"]))
        
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

        cv2.putText(img, label1, (x1, (y1-text2_height-baseline-2)), fontface, scale, (0, 0, 255), thickness, cv2.LINE_AA)
        if ENABLEDEPTH:
            cv2.putText(img, label2, (x1, int(y1-baseline/2 -1)), fontface, scale, (0, 0, 255), thickness, cv2.LINE_AA)

        cv2.rectangle(img, (x1, y1, x2, y2), colors, 2)
    
    tmp_string = "num: " + str(no_of_persons)
    cv2.putText(img, tmp_string, (0, 30), 0, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

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
	
	enableDepth = False
	enable3Dlocation = False
	
	PAL_PYTHON.SetDepthModeInTrackingP(PAL_PYTHON.DEPTH_OFFP)

	tracking_mode = PAL_PYTHON.PEOPLE_FOLLOWINGP
	success = PAL_PYTHON.SetModeInTrackingP(tracking_mode)
	
	# Creating a window
	source_window = 'PAL People Tracking'
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
	print("Press d/D to enable/Disable Depth calculation.")
	print("Press l/L to enable/Disable 3D Location calculation.")
	print("Press m/M to toggle Fast Depth property")
	print("Press i/I to set ID of the person you want to follow.")
	print("Press p/P to print the ID of the person being followed.\n\n")

	# ESC
	while key != 27:

		left, right, depth, trackingData, camera_changed =  PAL_PYTHON.GrabTrackingDataP()
		if camera_changed == True:
			break
		display = left
        
		drawOnImage(display, trackingData, tracking_mode, enableDepth, enable3Dlocation)
		
		cv2.imshow(source_window, display)
		
		#print_track(trackingData)

        	# Wait for 1ms
		key = cv2.waitKey(1) & 255

		#i
		if key == 105:	
			zenityCmd = "zenity --entry --text \"Enter ID to Track\" --title \"PAL People Following\" --entry-text=\"\""
			output_zenity = subprocess.run(zenityCmd, shell=True, stdout=subprocess.PIPE, universal_newlines=True)
			output = output_zenity.stdout
			output = output.replace(" ", "")
			output = output.replace("\n", "")
			output = output.replace("\t", "")
			if is_number(output):
				PAL_PYTHON.SetTrackIDP(int(output))
			else:
				errorCmd = "zenity --warning  --text \"Not a valid input\" --title \"PAL People Following\""
				error_output = subprocess.run(errorCmd, shell=True, stdout=subprocess.PIPE, universal_newlines=True)    
		
		#p
		if key == 112:	
			new_id = PAL_PYTHON.GetTrackIDP()	    
			if new_id == -2:
				print("Currently not in Following Mode")
			elif new_id == -1:
				print("No ID has been entered. Press 'i' to enter an ID")
			else:
				print("The ID currently being followed is: ",new_id)

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

		#d
		if key == 100:	
			enableDepth = not(enableDepth)	    
			if enableDepth:
				if enable3Dlocation:
					PAL_PYTHON.SetDepthModeInTrackingP(PAL_PYTHON.DEPTH_3DLOCATION_ONP)
				else:
					PAL_PYTHON.SetDepthModeInTrackingP(PAL_PYTHON.DEPTH_ONP)
			else:
				PAL_PYTHON.SetDepthModeInTrackingP(PAL_PYTHON.DEPTH_OFFP)

		#l
		if key == 108:	
 			enable3Dlocation = not(enable3Dlocation)

    	# Destroying connections
	print("exiting the application\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
	main()





