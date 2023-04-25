# CODE SAMPLE # 005: PAL Camera Properties
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
		
	width, height, ack_init = PAL_PYTHON.InitP(image_width, image_height, camera_index)

	if ack_init != PAL_PYTHON.SUCCESSP:
		print("Camera Init failed\n")
		return

	PAL_PYTHON.SetAPIModeP(PAL_PYTHON.DEPTHP)

	loaded_prop = {}
	prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
	
	loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", prop)
	if ack_load == PAL_PYTHON.INVALID_PROPERTY_VALUEP: 
		PAL_PYTHON.DestroyP()
		return
	
	
	if ack_load != PAL_PYTHON.SUCCESSP:
		print("Error Loading settings! Loading default values.")
	
	for i in range(0, 5):
		left, right, depth, raw_depth, camera_changed  = PAL_PYTHON.GrabDepthDataP()
	# Creating a window
	source_window = 'PAL Camera Properties'
	cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
	
	# Current image resolution
	#print("The image resolution is : ", width, "x", height, "\n")

	# Changing window size
	cv2.resizeWindow(source_window, (int(width), int((height)*2)))

	key = ' '

	print("\n\nPress ESC to close the window.")
	print("Q & A keys increase and decrease the BRIGHTNESS respectively.")
	print("W & S keys increase and decrease the CONTRAST respectively.")
	print("E & D keys increase and decrease the SATURATION respectively.")
	print("R & Z keys increase and decrease the GAMMA respectively.")
	print("T & G keys increase and decrease the GAIN respectively.")
	print("Y & H keys increase and decrease the WHITE BALANCE TEMPERATURE respectively.")
	print("U & J keys increase and decrease the SHARPNESS respectively.")
	print("I & K keys increase and decrease the EXPOSURE respectively.")
	print("O key toggles AUTO WHITE BALANCE property.")
	print("P key toggles AUTO EXPOSURE property.")
	print("C key saves the current left+depth panorama image to a numbered file.")
	print("N key saves the current camera properties to a file.")
	print("L key loads the camera properties from the saved file.")
	print("V & v keys toggles flip property.")	
	print("F & f keys toggles filter rgb property.\n\n")
 
	flip = False
	filter_spots = bool(loaded_prop["filter_spots"])
	pitch = int(loaded_prop["pitch"])
	frame = 0
	# ESC
	while key != 27:
		# GrabFrames function
		left, right, depth, _, camera_changed  = PAL_PYTHON.GrabDepthDataP()
		if camera_changed == True:
			break
		# BGR->RGB FLOAT->RGB
		depth_mat = np.uint8(depth)
		depth_mat = cv2.cvtColor(depth_mat, cv2.COLOR_GRAY2RGB)
		
		# Concatenate vertically
		display = cv2.vconcat([left,depth_mat])

		# Show results
		cv2.imshow(source_window, display)

		# Wait for 1ms
		key = cv2.waitKey(1) & 255
		#print(key)
		flags = 0
		if key == 32:
			loaded_prop["brightness"] = PAL_PYTHON.DEFAULT_BRIGHTNESSP
			loaded_prop["contrast"] = PAL_PYTHON.DEFAULT_CONTRASTP
			loaded_prop["saturation"] = PAL_PYTHON.DEFAULT_SATURATIONP
			loaded_prop["gamma"] = PAL_PYTHON.DEFAULT_GAMMAP
			loaded_prop["gain"] = PAL_PYTHON.DEFAULT_GAINP
			loaded_prop["white_bal_temp"] = PAL_PYTHON.DEFAULT_WHITE_BAL_TEMPP
			loaded_prop["sharpness"] = PAL_PYTHON.DEFAULT_SHARPNESSP
			loaded_prop["exposure"] = PAL_PYTHON.DEFAULT_EXPOSUREP
			loaded_prop["auto_white_bal"] = PAL_PYTHON.DEFAULT_AUTO_WHITE_BALP
			loaded_prop["auto_gain"] = PAL_PYTHON.DEFAULT_AUTO_GAINP
			flags = PAL_PYTHON.ALLP
			
		elif key == 113:
			loaded_prop["brightness"] += 1
			if loaded_prop["brightness"] > PAL_PYTHON.MAX_BRIGHTNESSP:
				loaded_prop["brightness"] = PAL_PYTHON.MAX_BRIGHTNESSP 
			flags |= PAL_PYTHON.BRIGHTNESSP
		elif key == 97:
			loaded_prop["brightness"] -= 1
			if loaded_prop["brightness"] < PAL_PYTHON.MIN_BRIGHTNESSP:
				loaded_prop["brightness"] = PAL_PYTHON.MIN_BRIGHTNESSP 
			flags |= PAL_PYTHON.BRIGHTNESSP
		
		elif key == 119:
			loaded_prop["contrast"] += 1
			if loaded_prop["contrast"] > PAL_PYTHON.MAX_CONTRASTP:
				loaded_prop["contrast"] = PAL_PYTHON.MAX_CONTRASTP 
			flags |= PAL_PYTHON.CONTRASTP
		elif key == 115:
			loaded_prop["contrast"] -= 1
			if loaded_prop["contrast"] < PAL_PYTHON.MIN_CONTRASTP:
				loaded_prop["contrast"] = PAL_PYTHON.MIN_CONTRASTP 
			flags |= PAL_PYTHON.CONTRASTP
		
		elif key == 101:
			loaded_prop["saturation"] += 1
			if loaded_prop["saturation"] > PAL_PYTHON.MAX_SATURATIONP:
				loaded_prop["saturation"] = PAL_PYTHON.MAX_SATURATIONP
			flags |= PAL_PYTHON.SATURATIONP
		
		elif key == 100:
			loaded_prop["saturation"] -= 1
			if loaded_prop["saturation"] < PAL_PYTHON.MIN_SATURATIONP:
				loaded_prop["saturation"] = PAL_PYTHON.MIN_SATURATIONP 
			flags |= PAL_PYTHON.SATURATIONP
		
		elif key == 114:
			loaded_prop["gamma"] += 10
			if loaded_prop["gamma"] > PAL_PYTHON.MAX_GAMMAP:
				loaded_prop["gamma"] = PAL_PYTHON.MAX_GAMMAP 
			flags |= PAL_PYTHON.GAMMAP
		
		elif key == 122:
			loaded_prop["gamma"] -= 10
			if loaded_prop["gamma"] < PAL_PYTHON.MIN_GAMMAP:
				loaded_prop["gamma"] = PAL_PYTHON.MIN_GAMMAP
			flags |= PAL_PYTHON.GAMMAP

		elif key == 116:
			loaded_prop["gain"] += 1
			if loaded_prop["gain"] > PAL_PYTHON.MAX_GAINP:
				loaded_prop["gain"] = PAL_PYTHON.MAX_GAINP 
			flags |= PAL_PYTHON.GAINP
		
		elif key == 103:
			loaded_prop["gain"] -= 1
			if loaded_prop["gain"] < PAL_PYTHON.MIN_GAINP:
				loaded_prop["gain"] = PAL_PYTHON.MIN_GAINP 
			flags |= PAL_PYTHON.GAINP
		
		elif key == 121:
			loaded_prop["white_bal_temp"] += 200
			if loaded_prop["white_bal_temp"] > PAL_PYTHON.MAX_WHITE_BAL_TEMPP:
				loaded_prop["white_bal_temp"] = PAL_PYTHON.MAX_WHITE_BAL_TEMPP 
			flags |= PAL_PYTHON.WHITE_BAL_TEMPP
		
		elif key == 104:
			loaded_prop["white_bal_temp"] -= 200
			if loaded_prop["white_bal_temp"] < PAL_PYTHON.MIN_WHITE_BAL_TEMPP:
				loaded_prop["white_bal_temp"] = PAL_PYTHON.MIN_WHITE_BAL_TEMPP 
			flags |= PAL_PYTHON.WHITE_BAL_TEMPP
		
		elif key == 117:
			loaded_prop["sharpness"] += 1
			if loaded_prop["sharpness"] > PAL_PYTHON.MAX_SHARPNESSP:
				loaded_prop["sharpness"] = PAL_PYTHON.MAX_SHARPNESSP 
			flags |= PAL_PYTHON.SHARPNESSP
		
		elif key == 106:
			loaded_prop["sharpness"] -= 1
			if loaded_prop["sharpness"] < PAL_PYTHON.MIN_SHARPNESSP:
				loaded_prop["sharpness"] = PAL_PYTHON.MIN_SHARPNESSP
			flags |= PAL_PYTHON.SHARPNESSP
		
		elif key == 105:
			loaded_prop["exposure"] += 5
			if loaded_prop["exposure"] > PAL_PYTHON.MAX_EXPOSUREP:
				loaded_prop["exposure"] = PAL_PYTHON.MAX_EXPOSUREP 
			flags |= PAL_PYTHON.EXPOSUREP
		
		elif key == 107:
			loaded_prop["exposure"] -= 5
			if loaded_prop["exposure"] < PAL_PYTHON.MIN_EXPOSUREP:
				loaded_prop["exposure"] = PAL_PYTHON.MIN_EXPOSUREP 
			flags |= PAL_PYTHON.EXPOSUREP
		
		elif key == 111:
			loaded_prop["auto_white_bal"] = not(loaded_prop["auto_white_bal"])
			flags |= PAL_PYTHON.AUTO_WHITE_BALP
		
		elif key == 112:
			loaded_prop["auto_gain"] = not(loaded_prop["auto_gain"])
			flags |= PAL_PYTHON.AUTO_GAINP
		
		elif key == 99:
			fileName="./pal_image_"+str(frame)+".png"
			frame += 1
			cv2.imwrite(fileName, display)
			
		elif key == 110:
			PAL_PYTHON.SavePropertiesP("properties.txt")
			print(">>>>>> SAVED THE PROPERTIES >>>>")
                
		elif key == 108:
			loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("properties.txt", prop)
		
		elif key == 102:
			loaded_prop["filter_spots"] = not(loaded_prop["filter_spots"])
			flags |= PAL_PYTHON.FILTER_SPOTSP

		elif key == 118:		    
			loaded_prop["vertical_flip"] = not(loaded_prop["vertical_flip"])
			flags |= PAL_PYTHON.VERTICAL_FLIPP
		
		if flags != 0:
			prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flags)
			print("Camera Properties....\n")
			print("brightness         = ", loaded_prop["brightness"])
			print("contrast           = ", loaded_prop["contrast"])
			print("saturation         = ", loaded_prop["saturation"])
			print("gamma              = ", loaded_prop["gamma"])
			print("gain               = ", loaded_prop["gain"])
			print("white_bal_temp     = ", loaded_prop["white_bal_temp"])
			print("sharpness          = ", loaded_prop["sharpness"])
			print("exposure           = ", loaded_prop["exposure"])
			print("auto_white_bal     = ", loaded_prop["auto_white_bal"])
			print("auto_gain          = ", loaded_prop["auto_gain"])
			print("filter_spots       = ", loaded_prop["filter_spots"])
			print("vertical_flip      = ", loaded_prop["vertical_flip"])

	# Destroying connections
	print("exiting the application\n")
	PAL_PYTHON.DestroyP()

	return

if __name__ == "__main__":
    main()




