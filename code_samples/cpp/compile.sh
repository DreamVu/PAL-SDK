#!/bin/bash

filenames=`ls ./*.cpp`
ARCH=$(uname -p)
UBUNTU_VERSION=$(lsb_release -sc)
GPU=$(sudo lshw -C display | grep vendor)
GPU_LIBS="-ludev"
GPIO_LIBS=""
NO_OF_GPU=0
if [[ $GPU =~ "Nvidia" || $GPU =~ "NVIDIA" ]]; then
    NO_OF_GPU=1
else    
    NO_OF_GPU=0
fi

if [ "$UBUNTU_VERSION" = "jammy" ]; then
	NO_OF_GPU=0
fi

if [ "$ARCH" = "aarch64" ] || [ $NO_OF_GPU -eq 1 ]; then 
	GPU_LIBS="-ludev -lcudart -L/usr/local/cuda/lib64 -lnvinfer  /usr/src/tensorrt/bin/common/logger.o"
fi	

if [ "$ARCH" = "aarch64" ]; then 
	GPIO_LIBS="-lJetsonGPIO -w"
fi

if [ "$#" -ge 1 ]
then
	filename=$1
	echo "[INFO] Compiling code sample $filename"    	
	g++ $filename -L/usr/local/lib -lPAL `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o ${filename%.*}.out -I../../include/ -lv4l2 -lpthread  -ludev -std=c++11 -lX11 $GPU_LIBS $GPIO_LIBS
else
	echo "[INFO] Compiling all the code samples!"    

	for eachfile in $filenames
	do
		echo "[INFO] Compiling code sample $eachfile"    	
		g++ $eachfile -L/usr/local/lib -lPAL `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o ${eachfile%.*}.out -I../../include/ -lv4l2 -lpthread  -ludev -std=c++11 -lX11 $GPU_LIBS $GPIO_LIBS
	done

fi
