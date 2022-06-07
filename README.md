# PAL USB
The only single sensor 360° 3D Vision System. [PAL USB](https://dreamvu.com/pal-usb/) is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

## System Requirements
* Jetpack 4.6

## Installation

The Package can be installed in two ways:

### Method 1. Using Debian packages

The Package can be downloaded directly from [here](https://github.com/DreamVu/ppa/raw/main/palusb/pal-usb.deb) and installed by running the below command,

    sudo apt install ./pal-usb.deb

### Method 2. Using PPA Repository

The Package can be installed by adding the PPA Repository. Steps are as follows:

#### Step 1. Adding DreamVu PPAs
    curl -SsL "https://dreamvu.github.io/ppa/KEY.gpg" | sudo apt-key add -
    sudo curl -SsL -o /etc/apt/sources.list.d/dreamvu.list "https://dreamvu.github.io/ppa/dreamvu.list"
    
#### Step 2. Installing PAL USB
    sudo apt update
    sudo apt install pal-usb


Once complete please reboot the system. The packages will be installed in \~/DreamVu folder. To preview the PAL USB camera run the below command 
    
    ~/DreamVu/PAL-USB/Explorer/Explorer

## Documentation 
For rest of the evaluation of the PAL-USB SDK, please read the [Evaluation Manual](https://docs.google.com/document/d/e/2PACX-1vT3rc_7S621PJHJ6QuV-rR2CyXbMvPBZztaDoiPnkT_g18Gz327OOA91pf11JMkqIeK0smel81rNbNg/pub)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc). 

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
