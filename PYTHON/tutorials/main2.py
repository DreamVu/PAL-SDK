# PAL TUTORIAL # 002:

# This tutorial shows the minimal code required to...
# 1. Change the camera properties
# 2. Write the panoramas captured with different camera properties

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

	properties = {}

	# Creating PAL::CameraProperties variable
	properties2 = PAL_PYTHON.createPALCameraPropertiesP(properties)

	# Modify the camera properties
	properties2["auto_white_bal"] = False
	properties2["white_bal_temp"] = PAL_PYTHON.MIN_WHITE_BAL_TEMPP
	
	# Changing Camera Properties
	properties3, flags_out, res1 = PAL_PYTHON.SetCameraPropertiesP(properties2)

	left = {}
	right = {}

	# Creating PAL::Image variables
	left1 = PAL_PYTHON.createPALImageP(left)
	right1 = PAL_PYTHON.createPALImageP(right)

	for i in range(10):
		# GrabFrames function
		left2, right2, depth2, disparity2, res  = PAL_PYTHON.GrabFramesP(left1, right1)
		left1 = left2
		right1 = right2

	# BGR->RGB
	left_mat = cv2.cvtColor(left2["u8_data"],cv2.COLOR_BGR2RGB)
	cv2.imwrite("left1.png", left_mat)

	# Modify the camera properties
	properties3["white_bal_temp"] = PAL_PYTHON.MAX_WHITE_BAL_TEMPP

	# Changing Camera Properties
	properties4, tmp1, res1 = PAL_PYTHON.SetCameraPropertiesP(properties3)
	
	for i in range(10):
		# GrabFrames function
		left3, right3, depth3, disparity3, res  = PAL_PYTHON.GrabFramesP(left2, right2)
		left2 = left3
		right2 = right3

	# BGR->RGB
	left_mat = cv2.cvtColor(left3["u8_data"],cv2.COLOR_BGR2RGB)
	cv2.imwrite("left2.png", left_mat)

	# Destroying connections
	print("Destroying...\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




