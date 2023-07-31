# CODE SAMPLE # 003: Range scan panorama
# This code will grab the left panorama with range scan overlayed on it and would be displayed in a window using opencv

import PAL_PYTHON
import cv2
import numpy as np
import sys

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
    PAL_PYTHON.SetAPIModeP(PAL_PYTHON.RANGE_SCANP)

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
    source_window = 'PAL Range Scan'
    cv2.namedWindow(source_window, cv2.WINDOW_AUTOSIZE)

    print("\n\nPress ESC to close the window.")
    print("Press v/V to flip vertically.\n\n")

    key = ' '
    
    # ESC
    while key != 27:
        # GrabFrames function
        scan_overlay_left, camera_changed = PAL_PYTHON.GrabRangeScanDataP()
        if camera_changed == True:
            break

        # Show results
        cv2.imshow(source_window, scan_overlay_left)

        # Wait for 1ms
        key = cv2.waitKey(1) & 255
        
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
