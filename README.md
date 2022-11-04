# PAL
The only single sensor 360° 3D Vision System. [PAL](https://dreamvu.com/pal-usb/) is the only single sensor omnidirectional vision system to provide 360° stereoscopic sensing with depth perception. 

## Pre-requisites Installation
 This section is only applicable for Ubuntu 18.04 intel system with Nvidia GPU.
 
 Please follow the steps on the official links of [CUDA](https://developer.nvidia.com/cuda-10.2-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=deblocal) 10.2 & [TensorRT](https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing-debian) to install the required GPU libraries.

## Installation

The Package can be installed in two ways:

### Method 1. Using Debian packages

The Package can be downloaded directly from [here](https://github.com/DreamVu/ppa/blob/main/pal/pal-temp?raw=true) and installed by running the below command from the location where it is downloaded,

    chmod +x pal-temp && ./pal-temp

### Method 2. Using PPA Repository

The Package can be installed by adding the PPA Repository. Steps are as follows:

#### Step 1. Adding DreamVu PPAs
    sudo wget -qO - https://dreamvu.github.io/ppa/KEY.gpg | sudo apt-key add -
    sudo wget -qO /etc/apt/sources.list.d/dreamvu.list https://dreamvu.github.io/ppa/dreamvu.list
    
#### Step 2. Installing PAL 
    sudo apt update
    sudo apt install ppa-pal
    sudo apt install pal-temp


Once complete please reboot the system. The packages will be installed in \~/DreamVu folder. 

To preview the PAL camera for x86_64 architecture run the below command 
 
    ~/DreamVu/PAL/Explorer/x86_64/Explorer
    
 To preview the PAL camera for ARM64 architecture run the below command
 	
    ~/DreamVu/PAL/Explorer/arm64/Explorer	 
    
## ROS Installations

### Ubuntu 18.04 supports ROS Melodic. 

#### To install ROS Melodic Navigation package use the below command:

    sudo apt install pal-melodic-navigation
    
### Ubuntu 20.04 supports ROS Noetic and ROS2 Foxy.

#### To install ROS Noetic Navigation package use the below command:

    sudo apt install pal-noetic-navigation

#### To install ROS2 Foxy Navigation package use the below command:

    sudo apt install pal-foxy-navigation
    
## Turtlebot-Nav
  
Turtlebot-Nav support is also provided for both the version and can be installed using following command,

    sudo apt install ros-melodic-turtlebot-nav    
      

## Documentation 
- [Setup Guide](https://docs.google.com/document/d/e/2PACX-1vQlEcAMp0yOgwLTvRZiYtiqG0MSRcgofzaiUCI_luNu4fQdQTowv8Cn7qHEgorE51ncfpGL7slKEQxJ/pub)
- [ROS Application Note](https://docs.google.com/document/d/e/2PACX-1vRrbqXkhQ5cnHNl_Idakk5dnGZ90bDOQk1Be2Jc-jlVJyCgOJZfUgyNGaco9sPDilcSS8gjk1wnR_dq/pub)
- [Design Integration Guide](https://docs.google.com/document/d/e/2PACX-1vTivOhDfGu0S9OJQ9klA4CMnAqKRzOjTXihousnYK1ukarJE1vM67VRXgHpJzVE00UqPGpq1Gb6_7qQ/pub)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc). 

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
