# PAL TUTORIAL # 001:

# This tutorial shows the minimal code required to...
# 1. Grab stereo panoramic images from PAL API
# 2. Write the OpenCV Mat into png files.

import PAL_PYTHON
import cv2
import numpy as np

def main():
	
	# This should be same as the camera index for OpenCV video capture.
	# When -1 is used, PAL API would try to automatically detect the index of PAL camera.
	camera_index = -1
	
	image_width = -1
	image_height = -1

	# Initialising camera
	image_width, image_height, res_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

	if res_init!= PAL_PYTHON.SUCCESSP:
		print("Unable to initialize PAL camera. Please make sure that the camera is connected\n")
		return # Init failed

	left = {}
	right = {}

	# Creating PAL::Image variables
	left = PAL_PYTHON.createPALImageP(left)
	right = PAL_PYTHON.createPALImageP(right)

	# GrabFramesP function
	left, right, depth, disparity, res  = PAL_PYTHON.GrabFramesP(left, right)

	# BGR->RGB
	left_mat = cv2.cvtColor(left["u8_data"],cv2.COLOR_BGR2RGB)
	right_mat = cv2.cvtColor(right["u8_data"],cv2.COLOR_BGR2RGB)
	
	# Saving images
	cv2.imwrite("left.png", left_mat)
	cv2.imwrite("right.png", right_mat)

	# Destroying connections
	print("Destroying...\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




