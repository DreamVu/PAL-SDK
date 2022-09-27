# CODE SAMPLE # 003: Range scan panorama
# This code will grab the left panorama with range scan overlayed on it and would be displayed in a window using opencv

import PAL_PYTHON
import cv2
import numpy as np
import sys

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
	
	width, height, res_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

	if res_init!= PAL_PYTHON.SUCCESSP:
		print("Camera Init failed\n")
		return

	PAL_PYTHON.SetAPIModeP(PAL_PYTHON.RANGE_SCANP)

	loaded_prop = {}
	prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
	
	loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", prop)

	if ack_load != PAL_PYTHON.SUCCESSP:
		print("Error Loading settings! Loading default values.")
	
	# Creating a window
	source_window = 'PAL Range Scan'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Current image resolution
	#print("The image resolution is : ", width, "x", height, "\n")

	# Changing window size
	cv2.resizeWindow(source_window, (int(width), int(height)))

	key = ' '

	print("Press ESC to close the window.")
	print("Press v/V to flip vertically.")

	flip = False
	pitch = int(loaded_prop["pitch"])
	
	# ESC
	while key != 27:

		# GrabFrames function
		rangescan  = PAL_PYTHON.GrabRangeScanDataP()

		# BGR->RGB
		rangescan_mat = cv2.cvtColor(rangescan,cv2.COLOR_BGR2RGB)

		# Show results
		cv2.imshow(source_window, rangescan_mat)

		# Wait for 1ms
		key = cv2.waitKey(1) & 255
		
		if key == 118:		    
			
			flag = PAL_PYTHON.PITCHP
						
			if flip == False:
				pitch = pitch-180
			else:
				pitch = pitch+180
				
			flip = not(flip)	
			loaded_prop["pitch"] = pitch
			prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)


	# Destroying connections
	print("exiting the application\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




