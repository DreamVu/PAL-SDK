# PAL TUTORIAL # 004:

# This tutorial shows the minimal code required to...
# 1. Compute the point cloud - based on the stereo panoramas
# 2. Save the point cloud into a file, that can be opened with tools like blender, meshlab etc.

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

	lst = []

	# Calculating point cloud and saving it
	point_cloud, timestamp_out, left, right, depth, disparity, res_getpc, res_savepc = PAL_PYTHON.GetPointCloudP(lst, "point_cloud.ply")

	if res_getpc != PAL_PYTHON.SUCCESSP:
		print("Unable to compute the point cloud\n")

	# Destroying connections
	print("Destroying...\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




