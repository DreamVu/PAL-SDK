# CODE SAMPLE # 001: Basic stereo panorama
# This code will grab the basic stereo panoramas (left and right images) and would be displayed in a window using opencv

import sys
import PAL_PYTHON
import cv2
import numpy as np
from Xlib.display import Display

def main():
    #Camera index is the video index assigned by the system to the camera. 
    #By default we set it to 5. Specify the index if the value has been changed.
    camera_index = 5    
    
    arg = len(sys.argv)
    if arg == 2:
        camera_index = int(sys.argv[1])
    
    #Connect to the PAL camera
    res_init = PAL_PYTHON.InitP(camera_index)

    if res_init!= PAL_PYTHON.SUCCESSP:
        print("Camera Init failed\n")
        return

    #Setting API Mode
    PAL_PYTHON.SetAPIModeP(PAL_PYTHON.STEREOP)
    
    loaded_prop = {}
    loaded_prop = PAL_PYTHON.createPALCameraPropertiesP(loaded_prop)
    
    #Loading camera properties from a text file
    loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("../../Explorer/SavedProperties.yml", loaded_prop)
    if ack_load == PAL_PYTHON.INVALID_PROPERTY_VALUEP: 
        PAL_PYTHON.DestroyP()
        return    
    if ack_load != PAL_PYTHON.SUCCESSP:
        print("Error Loading settings! Loading default values.")
    
    # Creating a window
    source_window = 'PAL Stereo Panorama'
    cv2.namedWindow(source_window, cv2.WINDOW_NORMAL)
    
    screen = Display().screen()
    sc_height = screen.height_in_pixels
    sc_width  = screen.width_in_pixels

    # Changing window size
    cv2.resizeWindow(source_window, sc_width-60, sc_height-60)
    
    filter_spots = bool(loaded_prop["filter_spots"])
    print("\n\nPress ESC to close the window.")
    print("Press f/F to toggle filter rgb property.")
    print("Press v/V to toggle vertical flip property.\n\n")

    key = ' '

    # ESC
    while key != 27:
        # GrabFrames function
        left, right, camera_changed  = PAL_PYTHON.GrabStereoDataP()
        if camera_changed == True:
            break

        # Concatenate vertically
        concat_op = cv2.vconcat([left,right])

        # Show results
        cv2.imshow(source_window, concat_op)

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
