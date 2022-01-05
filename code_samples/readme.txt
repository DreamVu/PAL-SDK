This folder contains six .cpp files, that serve as code samples - for displaying/saving/person_detections the panorama videos.

run compile.exe to compile all the six code samples.

After successful compilation, thre binaries should be created - with the same name as the .cpp files.

--> 001_stereo_panorama 
	Basic stereo panorama app, that shows left+right panoramas. 
	Press 'ESC' key to close the app.
    
--> 002_disparity_panorama 
	Basic stereo panorama app + DISPARITY, that shows left+right+disparity panoramas. 
	Press 'N' key to toggle disparity normalization
	Press f/F to toggle filter rgb property.
	Press v/V key to toggle the vertical flip of panorama
	Press 'ESC' key to close the app.
    
--> 003_video_capture 
	Same as the disparity panorama sample, but also provides video capture. 
	Press 'B' to begin video capture.
	Press 'E' to end the video capture, and close the window.
	Press 'C' to capture a single frame into a PNG file
	Press 'ESC' to close the window. If the video record has begun, it will end the capture before closing the app

--> 004_camera_properties
	Same as basic stero panorama, but allows users to modify the camera properties, load & save the camera properties and save snapshots to disk.
	Press 'ESC' key to close the app.
	Press 'Q' & 'A' keys to increase and decrease the SATURATION respectively.
	Press 'W' & 'S' keys to increase and decrease the GAMMA respectively.
	Press 'E' & 'D' keys to increase and decrease the GAIN respectively.
	Press 'R' & 'F' keys to increase and decrease the WHITE BALANCE TEMPERATURE respectively.
	Press 'T' key to toggle AUTO WHITE BALANCE property.
	Press 'Y' key to toggle AUTO EXPOSURE property.
	Press 'P' key to save the current left+right panorama image to a numbered file.
	Press 'O' key to save the current camera properties to a file. 
	Press 'L' key to load the camera properties from the saved file.

--> 005_resolutions
	This code sample allows users to access data proccessed at different resolutions and get pixel specific information on mouse clicks.
	Press 'N' key to toggle disparity normalization
	Press 'ESC' key to close the app.
	Press 1 to use highest resolution
	Press 2 to use second resolution
	Press 3 to use third of the resolution
	Press 4 to use lowest of the resolution
	Click 'Left mouse button' for RGB, disparity and depth information specifc to the pixel clicked
	
--> 006_person_detections
	This code sample allows users to learn how to use Person Detection APIs and further use the data to create a simple social distancing application
	Press ESC to close the window. 
	Press f/F to toggle filter rgb property.
	Press v/V key to toggle the vertical flip of panorama
	Press 1 to use highest resolution
	Press 2 to use second resolution
	Press 3 to use third of the resolution
	Press 4 to use lowest of the resolution	
