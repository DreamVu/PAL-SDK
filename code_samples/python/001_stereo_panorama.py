# CODE SAMPLE # 001: Basic stereo panorama
# This code will grab the basic stereo panoramas (left and right images) and would be displayed in a window using opencv

import sys
import PAL_PYTHON
import cv2
import numpy as np


def main():

	# Initialising camera
	image_width = 0
	image_height = 0
	camera_index = 5	
	arg = len(sys.argv)

	if arg == 2:
		camera_index = int(sys.argv[1])
	
	path = "/usr/local/bin/data/pal/data"+str(camera_index)+"/"	
	PAL_PYTHON.SetPathtoDataP(path)
	PAL_PYTHON.DisableTRTModelsP(True)	
	width, height, res_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

	

	if res_init!= PAL_PYTHON.SUCCESSP:
		print("Camera Init failed\n")
		return

	PAL_PYTHON.SetAPIModeP(PAL_PYTHON.STEREOP)
	
	loaded_prop = {}
	prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
	
	loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", prop)
	
	# Creating a window
	source_window = 'PAL Stereo Panorama'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Current image resolution
	#print("The image resolution is : ", width, "x", height, "\n")

	# Changing window size
	cv2.resizeWindow(source_window, (int(width*2), int(height*2)))

	key = ' '
	filter_spots = bool(loaded_prop["filter_spots"])
	print("Press ESC to close the window.\n")
	print("Press f/F to toggle filter rgb property.")

	# ESC
	while key != 27:

		# GrabFrames function
		left, right  = PAL_PYTHON.GrabStereoDataP()

		# BGR->RGB
		left_mat = cv2.cvtColor(left,cv2.COLOR_BGR2RGB)
		right_mat = cv2.cvtColor(right,cv2.COLOR_BGR2RGB)

		# Concatenate vertically
		concat_op = cv2.vconcat([left_mat,right_mat])

		# Show results
		cv2.imshow(source_window, concat_op)

		# Wait for 1ms
		key = cv2.waitKey(1) & 255
		
		if key == 102:		    
			flag = PAL_PYTHON.FILTER_SPOTSP
			filter_spots = not(filter_spots)
			loaded_prop["filter_spots"] = filter_spots
			prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)

	# Destroying connections
	print("exiting the application\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




