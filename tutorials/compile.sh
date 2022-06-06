g++ 001_cv_png.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DE.so ../lib/libPAL_EDET.so   `pkg-config --libs --cflags opencv`   -lv4l2 -lpthread -O3 -o 001_cv_png.out -I../include/

g++ 002_set_properties.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DE.so ../lib/libPAL_EDET.so  `pkg-config --libs --cflags opencv` -lv4l2 -lpthread  -O3 -o 002_set_properties.out -I../include/

g++ 003_point_cloud.cpp  /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_CAMERA.so   ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_DE.so ../lib/libPAL_EDET.so `pkg-config --libs --cflags opencv`   -lv4l2 -lpthread -O3 -o 003_point_cloud.out -I../include/

