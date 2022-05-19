# PAL USB
The only single sensor 360° 3D Vision System. [PAL USB](https://dreamvu.com/pal-usb/) is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

## System Requirements
* Jetpack 4.6

## Installation

The Package can be installed in two ways:

### &nbsp;&nbsp;&nbsp;&nbsp;Method 1. Using Debian packages

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The Package can be downloaded directly from [here](https://github.com/DreamVu/ppa/raw/main/palusb/palusb_3.4_arm64.deb) and installed by running the below command,
    
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  `sudo apt install ./palusb_jetpack_3.4_arm64.deb`
    
### &nbsp;&nbsp;&nbsp;&nbsp;Method 2. Using PPA Repository

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;The Package can be installed by adding the PPA Repository. Steps are as follows:

#### &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Step 1. Adding DreamVu PPAs
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `curl -SsL "https://dreamvu.github.io/ppa/KEY.gpg" | sudo apt-key add -`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `sudo curl -SsL -o /etc/apt/sources.list.d/dreamvu.list "https://dreamvu.github.io/ppa/dreamvu.list"`
    
#### &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Step 2. Installing PAL USB
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `sudo apt update`  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `sudo apt install pal-usb`

Once complete please reboot the system. The packages will be installed in \~/DreamVu folder. To preview the PAL USB camera run the below command 
    
`~/DreamVu/PAL-USB/Explorer/Explorer`

## Documentation 
For rest of the evaluation of the PAL-USB SDK, please read the [Evaluation Manual](https://docs.google.com/document/d/e/2PACX-1vT3rc_7S621PJHJ6QuV-rR2CyXbMvPBZztaDoiPnkT_g18Gz327OOA91pf11JMkqIeK0smel81rNbNg/pub)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc). 

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
