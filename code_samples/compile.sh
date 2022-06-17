 g++ 001_stereo_panorama.cpp  `pkg-config --libs --cflags opencv pal` -O3  -o 001_stereo_panorama.out

 g++ 002_disparity_panorama.cpp  `pkg-config --libs --cflags opencv pal` -O3  -o 002_disparity_panorama.out

g++ 003_video_capture.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 003_video_capture.out

g++ 004_camera_properties.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 004_camera_properties.out
 
g++ 005_person_detection.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 005_person_detection.out

g++  006_occupancy_map.cpp `pkg-config --libs --cflags opencv pal` -O3  -o  006_occupancy_map.out -lstdc++fs

g++  007_3d_person_locations.cpp `pkg-config --libs --cflags opencv pal` -O3  -o  007_3d_person_locations.out -lstdc++fs

g++  008_social_distancing.cpp  `pkg-config --libs --cflags opencv pal` -O3  -o 008_social_distancing.out -lstdc++fs

g++ 009_object_detection.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 009_object_detection.out

g++ 010_safe_zone_detection.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 010_safe_zone_detection.out

g++ 011_object_tracking.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 011_object_tracking.out

g++ 012_people_following.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 012_people_following.out

g++ 013_object_tracking_GPIO.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 013_object_tracking_GPIO.out -lJetsonGPIO -w

g++ 014_UART_sender.cpp `pkg-config --libs --cflags opencv pal` -O3  -o 014_UART_sender.out -lJetsonGPIO -w
