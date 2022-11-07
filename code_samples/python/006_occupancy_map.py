# CODE SAMPLE # 006: Occupancy Map
# This code sample allows users to access the region map within a depth range.

import sys
import PAL_PYTHON
import cv2
import numpy as np

def Getoccupancy1D(rgb_image, depth_mat, depth_thresh, context_threshold):
	ret, mask = cv2.threshold(depth_mat, depth_thresh, 255, cv2.THRESH_BINARY_INV)
	occupancySum = np.sum(mask.astype(np.float32), axis=0) / 255. 
	ret, occupancy1D = cv2.threshold(occupancySum, rgb_image.shape[0]*context_threshold/100.0, 255, cv2.THRESH_BINARY_INV);
	return (occupancy1D/255).astype(np.uint8).reshape(1,-1)

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
		
	width, height, ack_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

	if ack_init != PAL_PYTHON.SUCCESSP:
		print("Camera Init failed\n")
		return

	PAL_PYTHON.SetAPIModeP(PAL_PYTHON.DEPTHP)

	loaded_prop = {}
	prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
	
	loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", prop)

	if ack_load != PAL_PYTHON.SUCCESSP:
		print("Error Loading settings! Loading default values.")
	
	# Creating a window
	source_window = 'PAL Occupancy Map'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Current image resolution
	#print("The image resolution is : ", width, "x", height, "\n")

	# Changing window size
	cv2.resizeWindow(source_window, (int(width), int((height)*2)))

	key = ' '

	print("Press ESC to close the window.")
	print("Press v/V to flip vertically.")	
	print("Press f/F to toggle filter rgb property.")

	flip = bool(loaded_prop["vertical_flip"])
	filter_spots = bool(loaded_prop["filter_spots"])
	
	threshold_cm = 100
	context_threshold = 50
	
	# ESC
	while key != 27:
		# GrabFrames function
		left, right, depth, _  = PAL_PYTHON.GrabDepthDataP()

		# BGR->RGB FLOAT->RGB
		left_mat = cv2.cvtColor(left,cv2.COLOR_RGB2BGR)
		depth_mat = np.uint8(depth)
		
		occupancy1D = Getoccupancy1D(left_mat, depth_mat, threshold_cm, context_threshold)

		mask  = cv2.resize(occupancy1D, (left_mat.shape[1], left_mat.shape[0]), interpolation =cv2.INTER_NEAREST) * 255
		
		left_mat_c = left_mat.copy();
		left_mat_c[mask == 0] = (0,0,255)
		left_mat = (left_mat.astype(np.float32) * 0.7 + left_mat_c.astype(np.float32) * 0.3).astype(np.uint8) 
		
		# Show results		
		cv2.imshow(source_window, left_mat)

		# Wait for 1ms
		key = cv2.waitKey(1) & 255

		if key == 102:		    
			flag = PAL_PYTHON.FILTER_SPOTSP
			filter_spots = not(filter_spots)
			loaded_prop["filter_spots"] = filter_spots
			prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)
			
		if key == 118:		    
			flag = PAL_PYTHON.VERTICAL_FLIPP
			flip = not(flip)	
			loaded_prop["vertical_flip"] = flip
			prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)
		

	# Destroying connections
	print("exiting the application\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




