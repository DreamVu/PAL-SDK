# Initial Steps to follow

1. Unzip the SDK folder 

# SDK-Tutorials 

## PRE-REQUIREMENT: Install OpenCV and OpenCV-contrib modules
	1. Open docs/PAL Documentation.pdf file
	
	2. Follow the instructions as mentioned in the section 3.1 of the pdf file

	
## Install other dependencies and Compile the SDK Tutorials - This section is same as 3.2, 3.3 and 3.4  in the pdf file
1. Install QT, follow the process as described in the sec 3.1 in the documentation.
2. For installing python and other dependencies, Run install.exe
3. For the setup of Opencv and QT, Run opencv.exe 


# PAL USB
The only single sensor 360° 3D Vision System. [PAL USB](https://dreamvu.com/pal-usb/) is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

Please follow the instructions given below on any of the Intel x86 CPU to install the software.

## Step 1. Installing Dependencies 
Confirm the following dependencies. These are must have to proceed further

- ### Windows 10 64 bit
- ### Install OpenCV 3.4.4 and OpenCV Contrib 3.4.4 libraries. Go to the Installatios Folder and run opencv.exe 
-      
       chmod +x ./*.sh
       sudo ./opencv.sh

- ### Python 3.8 libraries (pytorch, tensorflow, numpy, PIL, etc.)
- ### QT 5.12.2

## Step 2. Installing QT 


## Step 2. Installing PAL USB SDK
      cd installations
      chmod +x ./*.sh
      sudo ./install.sh 

Once complete please reboot the system.

## Step 3. Installing libPAL_Camera.so
The libPAL_Camera.so file is delivered along with the purchase of the PAL USB camera. In case you have not received them, please request for the file by filling out a [form](https://support.dreamvu.com/portal/en/newticket). Place the libPAL_Camera.so file in the ./lib/ folder. 
      
## Documentation 
For rest of the evaluation of the PAL USB SDK, please read the [Evaluation Manual](https://github.com/DreamVu/PAL-USB/blob/Windows-10/docs/PAL%20SDK%20Windows%20Documentation.pdf)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc) or you can email us directly at support@dreamvu.com 

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
