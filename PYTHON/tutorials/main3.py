# PAL TUTORIAL # 003:

# This tutorial shows the minimal code required to...
# 1. Query the unnormalized 16 bit disparity.
# 2. Query the normalized 8 bit disparity.
# 3. Save the depth and disparity images

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

	# The disparity image would be single channel 8bit unsigned int data per pixel
	# Flag to indicate if the disparity image should be of 8bit normalized or 16 bit unnormalized
	normalize = True

	# Flag to indicate if the function should...
	# Return immediately (with the previous depth/disparity) or..
	# Should get blocked till all the latest depth/disparity computation is finished 
	asynchronous = False

	left = {}
	right = {}
	depth = {}
	disparity = {}

	# Creating PAL::Image variables
	left1 = PAL_PYTHON.createPALImageP(left)
	right1 = PAL_PYTHON.createPALImageP(right)
	depth1 = PAL_PYTHON.createPALImageP(depth)
	disparity1 = PAL_PYTHON.createPALImageP(disparity)

	# GrabFrames function
	left2, right2, depth2, disparity2, res  = PAL_PYTHON.GrabFramesP(left1, right1, depth1, disparity1, normalize, asynchronous)
	
	# Saving normalized disparity image
	disparity_mat = disparity2["u8_data"]
	cv2.imwrite("disparity_normalized.png", disparity_mat)

	# The disparity image would be single channel 16bit unsigned int data per pixel
	normalize = False

	# GrabFrames function
	left3, right3, depth3, disparity3, res2  = PAL_PYTHON.GrabFramesP(left2, right2, depth2, disparity2, normalize, asynchronous)

	# Saving unnormalized disparity image
	disparity_mat = disparity3["u16_data"]
	cv2.imwrite("disparity_unnormalized.png", disparity_mat)

	# Saving scaled depth image
	scale_factor = float(float(1.0) / float(1.6))
	depth4 = scale_factor*depth2["f32_data"]
	cv2.imwrite("depth.png", depth4)

	# Destroying connections
	print("Destroying...\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




