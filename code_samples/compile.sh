
g++ 001_stereo_panorama.cpp ../lib/libPAL.so ../lib/libPAL_CAMERA.so  ../lib/libPAL_DEPTH.so `pkg-config --libs --cflags opencv`   -O3  -o 001_stereo_panorama.out -I../include/ -lv4l2 -lpthread

g++ 002_disparity_panorama.cpp ../lib/libPAL.so ../lib/libPAL_CAMERA.so   ../lib/libPAL_DEPTH.so `pkg-config --libs --cflags opencv`    -O3  -o 002_disparity_panorama.out -I../include/ -lv4l2 -lpthread

g++ 003_video_capture.cpp ../lib/libPAL.so ../lib/libPAL_CAMERA.so    ../lib/libPAL_DEPTH.so `pkg-config --libs --cflags opencv`    -O3  -o 003_video_capture.out -I../include/ -lv4l2 -lpthread

g++ 004_camera_properties.cpp ../lib/libPAL.so ../lib/libPAL_CAMERA.so   ../lib/libPAL_DEPTH.so  `pkg-config --libs --cflags opencv`   -O3  -o 004_camera_properties.out -I../include/ -lv4l2 -lpthread

g++ 005_resolutions.cpp ../lib/libPAL.so ../lib/libPAL_CAMERA.so   ../lib/libPAL_DEPTH.so `pkg-config --libs --cflags opencv`   -O3  -o 005_resolutions.out -I../include/ -lv4l2 -lpthread

