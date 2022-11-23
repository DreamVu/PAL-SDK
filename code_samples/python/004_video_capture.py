# CODE SAMPLE # 004: PAL Video Capture
# This code will grab the left & depth panorama and display in a window using opencv

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
		
	width, height, ack_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

	if ack_init != PAL_PYTHON.SUCCESSP:
		print("Camera Init failed")
		return

	PAL_PYTHON.SetAPIModeP(PAL_PYTHON.DEPTHP)

	loaded_prop = {}
	prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
	
	loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", prop)

	if ack_load != PAL_PYTHON.SUCCESSP:
		print("Error Loading settings! Loading default values.")
	
	# Creating a window
	source_window = 'PAL Video Capture'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Current image resolution
	#print("The image resolution is : ", width, "x", height, "\n")

	# Changing window size
	cv2.resizeWindow(source_window, (int(width), int((height)*2)))

	key = ' '
	record = False
	closed = False

	print("Press ESC to close the window.")
	print("Press v/V to flip vertically.")	
	print("Press f/F to toggle filter rgb property.")
	print("Press C to capture a single frame into a PNG file.")
	print("Press B to begin the video capture.")
	print("Press E to end the video capture.")
	
	flip = bool(loaded_prop["vertical_flip"])
	filter_spots = bool(loaded_prop["filter_spots"])
	
	# ESC
	while closed != True:
		# GrabFrames function
		left, right, depth, _  = PAL_PYTHON.GrabDepthDataP()

		# BGR->RGB FLOAT->RGB
		left_mat = cv2.cvtColor(left,cv2.COLOR_BGR2RGB)
		depth_mat = np.uint8(depth)
		depth_mat = cv2.cvtColor(depth_mat, cv2.COLOR_GRAY2RGB)
		
		# Concatenate vertically
		output = cv2.vconcat([left_mat,depth_mat])

		# Show results
		cv2.imshow(source_window, output)

		# Wait for 1ms
		key = cv2.waitKey(1) & 255
		#print(key)
		
		if key == 99:
			cv2.imwrite("image.png", output)
		elif key == 98:
			fps = 15
			print("Opening the video.")
			video = cv2.VideoWriter("pal_video.avi",cv2.VideoWriter_fourcc('X','V','I','D'), fps, (output.shape[1],output.shape[0]))
			record = True
		elif key == 101 or key == 27:
			closed = True
			if record:
				record = False;           
				print("Releasing the video.")
				video.release();

		if record == True:
			video.write(output)
					
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
	print("exiting the application")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




