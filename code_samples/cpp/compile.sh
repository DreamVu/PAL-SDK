#!/bin/bash

flag=7

if [ "$#" -ge 1 ]
then
	if ! [[ "$1" =~ ^[0-9]+$ ]]
		then
		    echo "Valid input is:"
		    echo "./compile.sh input"
		    echo "{where input will range from 1 to 6 based on which code sample to compile}"
		    exit 0
		else
			flag=$1    
	fi
fi
	
ARCH=$(uname -p)
NO_OF_GPU=0

GPU=$(sudo lshw -C display | grep vendor)
if [[ $GPU =~ "Nvidia" || $GPU =~ "NVIDIA" ]]; then
    NO_OF_GPU=1
else    
    NO_OF_GPU=0
fi

UBUNTU_VERSION=$(lsb_release -sc)
if [ "$UBUNTU_VERSION" = "focal" ]; then
	NO_OF_GPU=0
fi

GPU_LIBS="-ludev"

if [ "$ARCH" = "aarch64" ] || [ $NO_OF_GPU -eq 1 ]; then 
	GPU_LIBS="-ludev -lcudart -L/usr/local/cuda/lib64 -lnvinfer  /usr/src/tensorrt/bin/common/logger.o"
fi	

if [ $flag -eq 1 ] || [ $flag -eq 7 ]; then
	g++ 001_stereo_panorama.cpp -L/usr/local/lib -lPAL `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 001_stereo_panorama.out -I../../include/ -lv4l2 -lpthread  -ludev -std=c++11  $GPU_LIBS
fi

if [ $flag -eq 2 ] || [ $flag -eq 7 ]; then	
	g++ 002_depth_panorama.cpp -L/usr/local/lib -lPAL `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 002_depth_panorama.out -I../../include/ -lv4l2 -lpthread  -ludev -std=c++11 $GPU_LIBS
fi	
	
if [ $flag -eq 3 ] || [ $flag -eq 7 ]; then	
	g++ 003_range_scan_panorama.cpp -L/usr/local/lib -lPAL `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 003_range_scan_panorama.out -I../../include/ -lv4l2 -lpthread  -ludev -std=c++11  $GPU_LIBS 
fi

if [ $flag -eq 4 ] || [ $flag -eq 7 ]; then	  
    g++ 004_video_capture.cpp -L/usr/local/lib -lPAL `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 004_video_capture.out -I../../include/ -lv4l2 -lpthread  -ludev -std=c++11 $GPU_LIBS
fi 

if [ $flag -eq 5 ] || [ $flag -eq 7 ]; then   
    g++ 005_camera_properties.cpp -L/usr/local/lib -lPAL `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 005_camera_properties.out -I../../include/ -lv4l2 -lpthread  -ludev -std=c++11 $GPU_LIBS
fi  
 
if [ $flag -eq 6 ] || [ $flag -eq 7 ]; then    
    g++ 006_occupancy_map.cpp -L/usr/local/lib -lPAL `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 006_occupancy_map.out -I../../include/ -lv4l2 -lpthread  -ludev -lX11 -std=c++11 $GPU_LIBS
fi	
