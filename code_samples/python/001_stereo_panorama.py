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
		
	width, height, res_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

	if res_init!= PAL_PYTHON.SUCCESSP:
		print("Camera Init failed\n")
		return

	PAL_PYTHON.SetAPIModeP(PAL_PYTHON.STEREOP)
	
	PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt")
	
	# Creating a window
	source_window = 'PAL Stereo Panorama'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Current image resolution
	#print("The image resolution is : ", width, "x", height, "\n")

	# Changing window size
	cv2.resizeWindow(source_window, (int(width*2), int(height*2)))

	key = ' '

	print("Press ESC to close the window.\n")

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

	# Destroying connections
	print("exiting the application\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




