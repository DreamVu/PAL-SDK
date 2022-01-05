# CODE SAMPLE # 001: Switching Resolutions
# This code sample allows users to access data proccessed at different resolutions.

import PAL_PYTHON
import cv2
import numpy as np

def main():

	# Creating a window
	source_window = 'PAL DETECTION'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Initialising camera
	image_width = 0
	image_height = 0
	width, height, res_init = PAL_PYTHON.InitP(image_width, image_height, -1)

	if res_init!= PAL_PYTHON.SUCCESSP:
		print("Camera Init failed\n")
		return

	# InitPersonDetection function
	res_initpd = PAL_PYTHON.InitPersonDetectionP()

	if res_initpd != PAL_PYTHON.SUCCESSP:
		print("Person Detection Init failed\n") 

	# Current image resolution
	print("The image resolution is : ", width, "x", height, "\n")

	# Changing window size
	cv2.resizeWindow(source_window, (int(width/4), int((height/4)*3)))

	key = ' '

	print("Press ESC to close the window.\n")
	print("Press v to flip vertically\n")
	print("Press 1/2/3/4 to change resolution\n")

	flip = False
	resolutions = PAL_PYTHON.GetAvailableResolutionsP()

	# ESC
	while key != 27:

		left3 = np.zeros((1,1,3),dtype= np.uint8)
		depth3 = np.zeros((1,1),dtype= np.float32)
		tmp1 = []

		# GetPersonDetection function
		left4, depth4, BoundingBoxes_np, timestamp_tmp, res2 = PAL_PYTHON.GetPeopleDetectionP(left3, depth3, tmp1)

		# BGR->RGB
		img = cv2.cvtColor(left4,cv2.COLOR_BGR2RGB)

		# Making bounding box in the image
		for i in range(BoundingBoxes_np.shape[0]):
			cv2.rectangle(img,(BoundingBoxes_np[i]["x1"],BoundingBoxes_np[i]["y1"]),(BoundingBoxes_np[i]["x2"],BoundingBoxes_np[i]["y2"]),(0,255,0),5)
		
		# Show results
		cv2.imshow(source_window, img)

		# Wait for 1ms
		key = cv2.waitKey(1) & 255

		# key = 'v'
		if key == 118:		    
			tmp_dict = {}
			prop = PAL_PYTHON.createPALCameraPropertiesP(tmp_dict)
			if flip == False:
				flip = True
			else:
				flip = False
			prop["vertical_flip"] = flip
			flag = PAL_PYTHON.VERTICAL_FLIPP
			# print(prop)
			prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(prop, flag)

		# key = '1'/'2'/'3'/'4'
		if key == 49 or key == 50 or key == 51 or key == 52:
			index = key - 49
			tmp_dict = {}
			prop = PAL_PYTHON.createPALCameraPropertiesP(tmp_dict)
			flag = PAL_PYTHON.RESOLUTIONP
			prop["resolution"] = {"width":resolutions[index]["width"],"height": resolutions[index]["height"]}
			prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(prop, flag)
			print("Changing the resolution to :",prop["resolution"]["width"],"x",prop["resolution"]["height"])

	# Destroying connections
	print("Destroying...\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




