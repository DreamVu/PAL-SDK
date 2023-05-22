# PAL SDK
DreamVu PAL family of cameras are the only single sensor 360° 3D Vision Systems. This SDK is compatible with the following DreamVu cameras:
- [PAL USB](https://dreamvu.com/pal-usb/)
- [PAL Mini](https://dreamvu.com/pal-mini/)

## Pre-requisites Installation
 This section is only applicable for Ubuntu 18.04 intel system with Nvidia GPU.
 
 Please follow the steps on the official links of [CUDA](https://developer.nvidia.com/cuda-10.2-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1804&target_type=deblocal) 10.2 & [TensorRT 8.X](https://developer.nvidia.com/nvidia-tensorrt-8x-download) to install the required GPU libraries. Please note that SDK has been tested for CUDA v10.2 & Tensorrt v8.0.0.3 combination. 

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

To preview the PAL camera, run the below command 
 
    Explorer
    
## ROS Installations

### Ubuntu 18.04 supports ROS Melodic. 

#### To install ROS Melodic Navigation package use the below command:

    sudo apt install pal-melodic-navigation-temp
    
### Ubuntu 20.04 supports ROS Noetic and ROS2 Foxy.

#### To install ROS Noetic Navigation package use the below command:

    sudo apt install pal-noetic-navigation-temp

#### To install ROS2 Foxy Navigation package use the below command:

    sudo apt install pal-foxy-navigation-temp
    
## Turtlebot-Nav
  
Turtlebot-Nav support is also provided for both the version and can be installed using following command,

    sudo apt install ros-melodic-turtlebot-nav    
      

## Documentation 
- [Setup Guide](https://docs.google.com/document/d/e/2PACX-1vSM_AQwWX1f3KgIoGIwkT_xsTBEleebKtY8i6gTaxIulw3gR0u_-wLhkp5Qxe2Janj6MUMx-rZxQf9-/pub)
- [Code Samples](https://docs.google.com/document/d/e/2PACX-1vTXnJwI1fN3Wh3HFpQNjwB-D29oUors_tbn9dFaQ-kKOT7m0S45OQTIK4eIkPt5gQFghnBCtNXx9hFw/pub)
- [PAL ROS Application Note](https://docs.google.com/document/d/e/2PACX-1vRrbqXkhQ5cnHNl_Idakk5dnGZ90bDOQk1Be2Jc-jlVJyCgOJZfUgyNGaco9sPDilcSS8gjk1wnR_dq/pub)
- [PAL Mini ROS Application Note](https://docs.google.com/document/d/e/2PACX-1vS8XpaUZAu6q5TRsJzVaWwDdjwRKgArtJ4zVdHj6nsrHrvVfGSlu3hm9ecHhCMaBqLlIYdlguVTJJH-/pub)
- [GPIO & UART Application Note](https://docs.google.com/document/d/e/2PACX-1vTN9U7ZocPkSLjN90oEgiOtFgr4e81qbgLsfpibcUGtQnvx3zpwMETmWvJ4BujKfcuOYSs_Yh95_4fm/pub)
- [Design Integration Guide](https://docs.google.com/document/d/e/2PACX-1vTzozqh7LtwgcBRXhxrZCy6jdk5TG6VzUCgNuZqZzNg5orSkilWFPm9WlGZ7PaZNOsGiVRC8i_-cXle/pub)

## Support 
If you need help or more informations check our [Help Center](https://support.dreamvu.com/portal/en/home) or join our [Community](https://support.dreamvu.com/portal/en/community/dreamvu-inc). 

## Contributing
Feel free to open an issue if you find a bug, or a pull request for bug fixes, features or other improvements.
