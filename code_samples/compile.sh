ARCH=$(uname -p)
NO_OF_GPU=0

GPU=$(sudo lshw -C display | grep vendor)
if [[ $GPU =~ "Nvidia" || $GPU =~ "NVIDIA" ]]; then
    NO_OF_GPU=1
else    
    NO_OF_GPU=0
fi
UBUNTU_VERSION=$(lsb_release -sc)
GPU_LIBS=""
PYTHON_LIBS=""

if [ "$ARCH" = "x86_64" ]; 
then
    if [ $NO_OF_GPU -eq 1 ]; 
    then
    	GPU_LIBS="-L/usr/local/cuda/lib64 -lnvinfer -lcudart /usr/src/tensorrt/bin/common/logger.o"	     
    else
    	GPU_LIBS=""
    fi
    
	g++ 001_stereo_panorama.cpp -L/usr/local/lib -lPAL_MINI `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 001_stereo_panorama.out -I../include/ -lv4l2 -lpthread  -ludev -std=c++11 $GPU_LIBS
	g++ 002_depth_panorama.cpp -L/usr/local/lib -lPAL_MINI `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 002_depth_panorama.out -I../include/ -lv4l2 -lpthread  -ludev -std=c++11 $GPU_LIBS
	g++ 003_range_scan_panorama.cpp -L/usr/local/lib -lPAL_MINI `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 003_range_scan_panorama.out -I../include/ -lv4l2 -lpthread  -ludev -std=c++11 $GPU_LIBS   
    g++ 004_video_capture.cpp -L/usr/local/lib -lPAL_MINI `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 004_video_capture.out -I../include/ -lv4l2 -lpthread  -ludev -std=c++11 $GPU_LIBS
    g++ 005_camera_properties.cpp -L/usr/local/lib -lPAL_MINI `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 005_camera_properties.out -I../include/ -lv4l2 -lpthread  -ludev -std=c++11 $GPU_LIBS
    g++ 006_occupancy_map.cpp -L/usr/local/lib -lPAL_MINI `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 006_occupancy_map.out -I../include/ -lv4l2 -lpthread  -ludev -lX11 -std=c++11 $GPU_LIBS
    
else
	if [ "$UBUNTU_VERSION" = "focal" ];
	then 
	    PYTHON_LIBS="-L/usr/lib/python3.8/config-3.8-x86_64-linux-gnu -L/usr/lib -lpython3.8 -lcrypt -lpthread -ldl  -lutil -lm -lm"
	else
	    PYTHON_LIBS=""
	fi
	    
	g++ 001_stereo_panorama.cpp -L/usr/local/lib -lPAL_MINI /usr/src/tensorrt/bin/common/logger.o `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 001_stereo_panorama.out -I../include/ -lv4l2 -lpthread  -lcudart -L/usr/local/cuda/lib64 -lnvinfer -ludev -std=c++11 $PYTHON_LIBS
	g++ 002_depth_panorama.cpp -L/usr/local/lib -lPAL_MINI /usr/src/tensorrt/bin/common/logger.o `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 002_depth_panorama.out -I../include/ -lv4l2 -lpthread  -lcudart -L/usr/local/cuda/lib64 -lnvinfer -ludev -std=c++11 $PYTHON_LIBS
	g++ 003_range_scan_panorama.cpp -L/usr/local/lib -lPAL_MINI /usr/src/tensorrt/bin/common/logger.o `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 003_range_scan_panorama.out -I../include/ -lv4l2 -lpthread  -lcudart -L/usr/local/cuda/lib64 -lnvinfer -ludev -std=c++11 $PYTHON_LIBS
	g++ 004_video_capture.cpp -L/usr/local/lib -lPAL_MINI /usr/src/tensorrt/bin/common/logger.o `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 004_video_capture.out -I../include/ -lv4l2 -lpthread  -lcudart -L/usr/local/cuda/lib64 -lnvinfer -ludev -std=c++11 $PYTHON_LIBS
	g++ 005_camera_properties.cpp -L/usr/local/lib -lPAL_MINI /usr/src/tensorrt/bin/common/logger.o `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 005_camera_properties.out -I../include/ -lv4l2 -lpthread  -lcudart -L/usr/local/cuda/lib64 -lnvinfer -ludev -std=c++11 $PYTHON_LIBS
	g++ 006_occupancy_map.cpp -L/usr/local/lib -lPAL_MINI /usr/src/tensorrt/bin/common/logger.o `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 006_occupancy_map.out -I../include/ -lv4l2 -lpthread  -lcudart -L/usr/local/cuda/lib64 -lnvinfer -ludev -lX11 -std=c++11 $PYTHON_LIBS
fi
