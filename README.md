# PAL USB
The only single sensor 360° 3D Vision System. [PAL USB](https://dreamvu.com/pal-usb/) is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

Please follow the instructions given below on any of the Nvidia Jetson embedded boards with Jetpack 4.4.1 to install the software.

## Step 1. Clone the repository 
-     sudo apt-get install git-lfs
      git clone https://github.com/DreamVu/PAL-USB.git
      cd PAL-USB
      git lfs pull
      
## Step 2. Installing Dependencies 
Confirm the following dependencies. These are must have to proceed further

- ### Jetpack 4.4.1

- ### CUDA enabled OpenCV 4.4.0 and OpenCV Contrib 4.4.0 libraries. 
  Follow these steps to install the required OpenCV dependencies. 
-      cd installations
       chmod +x ./*.sh
       sudo ./opencv.sh

- ### Python 3.6 libraries (pytorch, torchvision, numpy, PIL, etc.)

## Step 3. Installing PAL USB SDK
      cd installations
      chmod +x ./*.sh
      sudo ./install.sh 

Once complete please reboot the system.

## Step 4. Installing Camera Data File 
The data files are delivered along with the purchase of the PAL USB camera. In case you have not received them, please request for the files by filling out a [form](https://support.dreamvu.com/portal/en/newticket)

      chmod +x setup.sh
      sudo ./setup.sh


## Documentation 
For rest of the evaluation of the PAL-USB SDK, please read the [Evaluation Manual](https://docs.google.com/document/d/e/2PACX-1vT3rc_7S621PJHJ6QuV-rR2CyXbMvPBZztaDoiPnkT_g18Gz327OOA91pf11JMkqIeK0smel81rNbNg/pub)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc) or you can email us directly at support@dreamvu.com 

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
