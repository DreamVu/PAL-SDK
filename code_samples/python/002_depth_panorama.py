# CODE SAMPLE # 002: Depth panorama
# This code will grab the left & depth panorama and display in a window using opencv

import sys
import PAL_PYTHON
import cv2
import numpy as np

def main():
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
    source_window = 'PAL Depth Panorama'
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
        if bool(loaded_prop["raw_depth"]):
            depth_mat = raw_depth
        else:
            depth_mat = depth
        
        depth_mat, color_add_ack = PAL_PYTHON.ColorDepthPostProcessingP(depth_mat)
        depth_mat = cv2.cvtColor(depth_mat, cv2.COLOR_BGR2RGB)

        # Concatenate vertically
        concat_op = cv2.vconcat([left,depth_mat])

        # Show results
        cv2.imshow(source_window, concat_op)

        # Wait for 1ms
        key = cv2.waitKey(1) & 255
        #print(key)

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
