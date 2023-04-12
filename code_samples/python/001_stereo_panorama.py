# CODE SAMPLE # 001: Basic stereo panorama
# This code will grab the basic stereo panoramas (left and right images) and would be displayed in a window using opencv

import sys
import PAL_PYTHON
import cv2
import numpy as np
from Xlib.display import Display

def main():

	# Initialising camera
	image_width = 0
	image_height = 0
	camera_index = 5	
	arg = len(sys.argv)

	if arg == 2:
		camera_index = int(sys.argv[1])
	
	PAL_PYTHON.DisableModelsP(True)	
	width, height, res_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

	

	if res_init!= PAL_PYTHON.SUCCESSP:
		print("Camera Init failed\n")
		return

	PAL_PYTHON.SetAPIModeP(PAL_PYTHON.STEREOP)
	
	loaded_prop = {}
	prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
	
	loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", prop)
	if ack_load == PAL_PYTHON.INVALID_PROPERTY_VALUEP: 
		PAL_PYTHON.DestroyP()
		return
		
	if ack_load != PAL_PYTHON.SUCCESSP:
		print("Error Loading settings! Loading default values.")	
	
	for i in range(0, 5):
		left, right, camera_changed  = PAL_PYTHON.GrabStereoDataP()
	
	# Creating a window
	source_window = 'PAL Stereo Panorama'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Current image resolution
	#print("The image resolution is : ", width, "x", height, "\n")
	
	screen = Display().screen()
	sc_height = screen.height_in_pixels;
	sc_width  = screen.width_in_pixels;
	

	# Changing window size
	cv2.resizeWindow(source_window, sc_width-60, sc_height-60)
	
	key = ' '
	filter_spots = bool(loaded_prop["filter_spots"])
	print("\n\nPress ESC to close the window.")
	print("Press f/F to toggle filter rgb property.\n\n")

	# ESC
	while key != 27:

		# GrabFrames function
		left, right, camera_changed  = PAL_PYTHON.GrabStereoDataP()
		if camera_changed == True:
			break

		# Concatenate vertically
		concat_op = cv2.vconcat([left,right])

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




