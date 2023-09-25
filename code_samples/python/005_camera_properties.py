# CODE SAMPLE # 005: PAL Camera Properties
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
    source_window = 'PAL Camera Properties'
    cv2.namedWindow(source_window, cv2.WINDOW_AUTOSIZE)

    print("\n\nPress ESC to close the window.")
    print("X & M keys increase and decrease the BRIGHTNESS respectively.")
    print("W & S keys increase and decrease the CONTRAST respectively.")
    print("E & D keys increase and decrease the SATURATION respectively.")
    print("R & Z keys increase and decrease the GAMMA respectively.")
    print("T & G keys increase and decrease the GAIN respectively.")
    print("Y & H keys increase and decrease the WHITE BALANCE TEMPERATURE respectively.")
    print("U & J keys increase and decrease the SHARPNESS respectively.")
    print("I & K keys increase and decrease the EXPOSURE respectively.")
    print("B & Q keys increase and decrease the FOCUS respectively.")
    print(", & . keys increase and decrease the HUE respectively.")
    print("O key toggles AUTO WHITE BALANCE property.")
    print("P key toggles AUTO EXPOSURE property.")
    print("A key toggles AUTO FOCUS property.")
    print("C key saves the current left+depth panorama image to a numbered file.")
    print("N key saves the current camera properties to a file.")
    print("L key loads the camera properties from the saved file.")
    print("V keys toggles flip property.")    
    print("F keys toggles filter rgb property.")
    print("Space key sets the default properties.\n\n")
 
    frame = 0

    key = ' '
    
    # ESC
    while key != 27:
        # GrabFrames function
        left, right, depth, raw_depth, camera_changed  = PAL_PYTHON.GrabDepthDataP()
        if camera_changed == True:
            break

        # BGR->RGB
        if bool(loaded_prop["raw_depth"]):
            depth_mat = raw_depth
        else:
            depth_mat = depth
            
        depth_mat, color_add_ack = PAL_PYTHON.ColorDepthPostProcessingP(depth_mat)
        depth_mat = cv2.cvtColor(depth_mat, cv2.COLOR_BGR2RGB)

        # Concatenate vertically
        display = cv2.vconcat([left,depth_mat])

        # Show results
        cv2.imshow(source_window, display)

        # Wait for 1ms
        key = cv2.waitKey(1) & 255
        

        flags = 0
        if key == 32:
            loaded_prop, default_prop_ack = PAL_PYTHON.SetDefaultCameraPropertiesP(loaded_prop)
            
        elif key == 120:
            loaded_prop["brightness"] += 1
            if loaded_prop["brightness"] > PAL_PYTHON.MAX_BRIGHTNESSP:
                loaded_prop["brightness"] = PAL_PYTHON.MAX_BRIGHTNESSP 
            flags |= PAL_PYTHON.BRIGHTNESSP
        elif key == 109:
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
            
        elif key == 98:
            loaded_prop["focus"] += 5
            if loaded_prop["focus"] > PAL_PYTHON.MAX_FOCUSP:
                loaded_prop["focus"] = PAL_PYTHON.MAX_FOCUSP 
            flags |= PAL_PYTHON.FOCUSP
        
        elif key == 113:
            loaded_prop["focus"] -= 5
            if loaded_prop["focus"] < PAL_PYTHON.MIN_FOCUSP:
                loaded_prop["focus"] = PAL_PYTHON.MIN_FOCUSP 
            flags |= PAL_PYTHON.FOCUSP
            
        elif key == 44:
            loaded_prop["hue"] += 5
            if loaded_prop["hue"] > PAL_PYTHON.MAX_HUEP:
                loaded_prop["hue"] = PAL_PYTHON.MAX_HUEP 
            flags |= PAL_PYTHON.HUEP
        
        elif key == 46:
            loaded_prop["hue"] -= 5
            if loaded_prop["hue"] < PAL_PYTHON.MIN_HUEP:
                loaded_prop["hue"] = PAL_PYTHON.MIN_HUEP 
            flags |= PAL_PYTHON.HUEP
        
        elif key == 111:
            loaded_prop["auto_white_bal"] = not(loaded_prop["auto_white_bal"])
            flags |= PAL_PYTHON.AUTO_WHITE_BALP
        
        elif key == 112:
            loaded_prop["auto_gain"] = not(loaded_prop["auto_gain"])
            flags |= PAL_PYTHON.AUTO_GAINP
            
        elif key == 97:
            loaded_prop["auto_focus"] = not(loaded_prop["auto_focus"])
            flags |= PAL_PYTHON.AUTO_FOCUSP
        
        elif key == 99:
            fileName="./pal_image_"+str(frame)+".png"
            frame += 1
            cv2.imwrite(fileName, display)
            
        elif key == 110:
            PAL_PYTHON.SavePropertiesP("properties.txt")
            print(">>>>>> SAVED THE PROPERTIES >>>>")
                
        elif key == 108:
            loaded_prop, ack_load = PAL_PYTHON.LoadPropertiesP("properties.txt", loaded_prop)
        
        elif key == 102:
            loaded_prop["filter_spots"] = not(loaded_prop["filter_spots"])
            flags |= PAL_PYTHON.FILTER_SPOTSP

        elif key == 118:            
            loaded_prop["vertical_flip"] = not(loaded_prop["vertical_flip"])
            flags |= PAL_PYTHON.VERTICAL_FLIPP
        
        if flags != 0:
            loaded_prop, flags, res_scp = PAL_PYTHON.SetCameraPropertiesP(loaded_prop, flags)
            print("Camera Properties....\n")
            print("brightness         = ", loaded_prop["brightness"])
            print("contrast           = ", loaded_prop["contrast"])
            print("saturation         = ", loaded_prop["saturation"])
            print("gamma              = ", loaded_prop["gamma"])
            print("gain               = ", loaded_prop["gain"])
            print("white_bal_temp     = ", loaded_prop["white_bal_temp"])
            print("sharpness          = ", loaded_prop["sharpness"])
            print("exposure           = ", loaded_prop["exposure"])
            print("focus              = ", loaded_prop["focus"])
            print("hue                = ", loaded_prop["hue"])
            print("auto_white_bal     = ", loaded_prop["auto_white_bal"])
            print("auto_gain          = ", loaded_prop["auto_gain"])
            print("auto_focus         = ", loaded_prop["auto_focus"])
            print("filter_spots       = ", loaded_prop["filter_spots"])
            print("vertical_flip      = ", loaded_prop["vertical_flip"])

    # Destroying connections
    print("exiting the application\n")
    PAL_PYTHON.DestroyP()

    return

if __name__ == "__main__":
    main()




