# CODE SAMPLE # 002: Basic stereo panorama
# This code will grab the basic stereo panoramas (left and right images) and would be displayed in a window using opencv

import PAL_PYTHON
import cv2
import numpy as np

def main():

	# Creating a window
	source_window = 'PAL'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Initialising camera
	image_width = 0
	image_height = 0
	width, height, res_init = PAL_PYTHON.InitP(image_width, image_height, -1)

	if res_init!= PAL_PYTHON.SUCCESSP:
		print("Camera Init failed\n")
		return

	# Current image resolution
	print("The image resolution is : ", width, "x", height, "\n")

	# Changing window size
	cv2.resizeWindow(source_window, (int(width/4), int((height/4)*2)))

	key = ' '

	print("Press ESC to close the window.\n")

	# ESC
	while key != 27:

		left = {}
		right = {}

		# Creating PAL::Image variables
		left1 = PAL_PYTHON.createPALImageP(left)
		right1 = PAL_PYTHON.createPALImageP(right)

		# GrabFrames function
		left2, right2, depth2, disparity2, res  = PAL_PYTHON.GrabFramesP(left1, right1)

		# BGR->RGB
		left_mat = cv2.cvtColor(left2["u8_data"],cv2.COLOR_BGR2RGB)
		right_mat = cv2.cvtColor(right2["u8_data"],cv2.COLOR_BGR2RGB)

		# Concatenate vertically
		concat_op = cv2.vconcat([left_mat,right_mat])

		# Show results
		cv2.imshow(source_window, concat_op)

		# Wait for 1ms
		key = cv2.waitKey(1) & 255

	# Destroying connections
	print("Destroying...\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




