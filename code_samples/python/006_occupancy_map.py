# CODE SAMPLE # 006: Occupancy Map
# This code sample allows users to access the region map within a depth range.

import sys
import PAL_PYTHON
import cv2
import numpy as np

def GetUserInput(prompt):
    while True:
        try:
            value = int(input(prompt))
            return value
        except ValueError:
            print("Invalid input. Please enter an integer.")

def Getoccupancy1D(rgb_image, depth_mat, depth_thresh, context_threshold):
    ret, mask = cv2.threshold(depth_mat, depth_thresh, 255, cv2.THRESH_BINARY_INV)
    occupancySum = np.sum(mask.astype(np.float32), axis=0) / 255. 
    ret, occupancy1D = cv2.threshold(occupancySum, rgb_image.shape[0]*context_threshold/100.0, 255, cv2.THRESH_BINARY_INV);
    return (occupancy1D/255).astype(np.uint8).reshape(1,-1)

def main():
    #depth threshold in cm
    #The depth threshold should be kept within 1m to 2m range.
    threshold_cm = GetUserInput("Enter the depth threshold in cm, eg 100\n")
    if(threshold_cm > 200):
        threshold_cm = 200
        print("depth threshold set above maximum range. Setting to 2m")
    elif(threshold_cm < 100):
        threshold_cm = 100
        print("depth threshold set below minimum range. Setting to 1m")

    #context threshold in percentage to be considered for occupancy
    #The context threshold should be kept within 50(recommended) to 80 range.
    context_threshold = GetUserInput("Enter the Context threshold. It is the percentage to be considered for occupancy, eg 50\n")
    if(context_threshold > 80):
        context_threshold = 80
        print("context threshold set above maximum range. Setting to 80")
    elif(context_threshold < 50):
        context_threshold = 50
        print("context threshold set below miminum range. Setting to 50")

    #Camera index is the video index assigned by the system to the camera. 
    #By default we set it to 5. Specify the index if the value has been changed.
    camera_index = 5    
    
    arg = len(sys.argv)
    if arg == 2:
        camera_index = int(sys.argv[1])

    #Connect to the PAL camera    
    res_init = PAL_PYTHON.InitP(camera_index)

    if res_init != PAL_PYTHON.SUCCESSP:
        print("Camera Init failed\n")
        return

    #Setting API Mode
    PAL_PYTHON.SetAPIModeP(PAL_PYTHON.DEPTHP)

    loaded_prop = {}
    loaded_prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
    
    #Loading camera properties from a text file
    loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedPalProperties.txt", loaded_prop)
    if ack_load == PAL_PYTHON.INVALID_PROPERTY_VALUEP: 
        PAL_PYTHON.DestroyP()
        return
    if ack_load != PAL_PYTHON.SUCCESSP:
        print("Error Loading settings! Loading default values.")

    # Creating a window
    source_window = 'PAL Occupancy Map'
    cv2.namedWindow(source_window, cv2.WINDOW_AUTOSIZE)
    
    print("\n\nPress ESC to close the window.")
    print("Press v/V to flip vertically.")    
    print("Press f/F to toggle filter rgb property.\n\n")

    key = ' '
    
    # ESC
    while key != 27:
        # GrabFrames function
        left, right, depth, raw_depth, camera_changed  = PAL_PYTHON.GrabDepthDataP()
        if camera_changed == True:
            break

        # FLOAT->RGB
        left_mat = left
        if bool(loaded_prop["raw_depth"]):
            depth_mat = raw_depth
        else:
            depth_mat = depth
        
        occupancy1D = Getoccupancy1D(left_mat, depth_mat, threshold_cm, context_threshold)
        mask  = cv2.resize(occupancy1D, (left_mat.shape[1], left_mat.shape[0]), interpolation =cv2.INTER_NEAREST) * 255
        
        color_mask = left_mat.copy();
        color_mask[mask == 0] = (0,0,255)
        left_mat = (left_mat.astype(np.float32) * 0.7 + color_mask.astype(np.float32) * 0.3).astype(np.uint8) 
        
        # Show results        
        cv2.imshow(source_window, left_mat)

        # Wait for 1ms
        key = cv2.waitKey(1) & 255

        if key == 102:            
            flag = PAL_PYTHON.FILTER_SPOTSP
            loaded_prop["filter_spots"] = not(bool(loaded_prop["filter_spots"]))
            loaded_prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)

        if key == 118:            
            flag = PAL_PYTHON.VERTICAL_FLIPP
            loaded_prop["vertical_flip"] = not(bool(loaded_prop["vertical_flip"]))
            loaded_prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flag)

    # Destroying connections
    print("exiting the application\n")
    PAL_PYTHON.DestroyP()

    return

if __name__ == "__main__":
    main()
