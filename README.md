# PAL USB
The only single sensor 360° 3D Vision System. [PAL USB](https://dreamvu.com/pal-usb/) is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

## System Requirements
* Jetpack 4.6

## Installation

The Package can be installed in two ways:

### Method 1. Using Debian packages

The Package can be downloaded directly from [here](https://github.com/DreamVu/ppa/blob/main/pal-usb/pal-usb?raw=true) and installed by running the below command from the location where it is downloaded,

    chmod +x pal-usb && ./pal-usb

### Method 2. Using PPA Repository

The Package can be installed by adding the PPA Repository. Steps are as follows:

#### Step 1. Adding DreamVu PPAs
    sudo wget -qO - https://dreamvu.github.io/ppa/KEY.gpg | sudo apt-key add -
    sudo wget -qO /etc/apt/sources.list.d/dreamvu.list https://dreamvu.github.io/ppa/dreamvu.list
    
#### Step 2. Installing PAL USB
    sudo apt update
    sudo apt install pal-usb


Once complete please reboot the system. The packages will be installed in \~/DreamVu folder. To preview the PAL USB camera run the below command 
    
    ~/DreamVu/PAL-USB/Explorer/Explorer

## ROS Melodic Installations

The Package can be downloaded directly from [here](https://github.com/DreamVu/ppa/blob/main/common/ros-melodic-dvu?raw=true) and installed by running the below command from the location where it is downloaded,

    chmod +x ros-melodic-dvu && ./ros-melodic-dvu

## Documentation 
- [Setup Guide](https://docs.google.com/document/d/e/2PACX-1vQvz5Ms3WfdZyYbyvzPIqjyyGyXEdKc9-4SrJUJa0WAnXnlB5WTftUWCWAfgfV6Xbaqmxh0S25kFsTu/pub)
- [Code Samples](https://docs.google.com/document/d/e/2PACX-1vQ32SF48KvE6VvsHW94KX-23A8a3baPhGHiT65xJ2AeR5tbdD72ZcA02KOfkAmuUBnsYT8Wf7iR5DlE/pub)
- [ROS Application Note](https://docs.google.com/document/d/e/2PACX-1vTN9U7ZocPkSLjN90oEgiOtFgr4e81qbgLsfpibcUGtQnvx3zpwMETmWvJ4BujKfcuOYSs_Yh95_4fm/pub)
- [GPIO & UART Application Note](https://docs.google.com/document/d/e/2PACX-1vTN9U7ZocPkSLjN90oEgiOtFgr4e81qbgLsfpibcUGtQnvx3zpwMETmWvJ4BujKfcuOYSs_Yh95_4fm/pub)
- [API Documentation](https://docs.google.com/document/d/e/2PACX-1vSrv7mq6tkspG0TiyJVDDMPw5PTJAG6Ecv41tnu8IERx5wKkCf7xXf26-udCOf1WvpbKkAmcXr-1UgT/pub)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc). 

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
