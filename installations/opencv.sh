sudo apt-get update -y
sudo apt-get install -y libjasper-dev, python3.6-dev, python3.6-venv, libglew2.0

sudo mkdir -p /usr/local/lib/pkgconfig/
sudo cp opencv.pc /usr/local/lib/pkgconfig/

cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.4.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.4.0.zip

unzip opencv.zip
unzip opencv_contrib.zip

mv opencv-4.4.0 opencv
mv opencv_contrib-4.4.0 opencv_contrib

cd ~/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_CUDA=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D BUILD_libwebp=OFF \
-D BUILD_libwebp=OFF \
-D BUILD_libjasper=OFF \
-D BUILD_IlmImf=OFF \
-D BUILD_libprotobuf=OFF \
-D BUILD_quirc=OFF \
-D BUILD_carotene_objs=OFF \
-D BUILD_tegra_hal=OFF \
-D BUILD_ittnotify=OFF \
-D BUILD_ade=OFF \
-D BUILD_opencv_videoio_plugins=OFF \
-D BUILD_opencv_cudev=ON \
-D BUILD_opencv_core=ON \
-D BUILD_opencv_imgproc=ON \
-D BUILD_opencv_imgcodecs=ON \
-D BUILD_opencv_videoio=ON \
-D BUILD_opencv_highgui=ON \
-D BUILD_opencv_ts=OFF \
-D BUILD_opencv_test_cudev=OFF \
-D BUILD_opencv_test_core=OFF \
-D BUILD_opencv_perf_core=OFF \
-D BUILD_opencv_cudaarithm=OFF \
-D BUILD_opencv_test_cudaarithm=OFF \
-D BUILD_opencv_perf_cudaarithm=OFF \
-D BUILD_opencv_flann=ON \
-D BUILD_opencv_test_flann=OFF \
-D BUILD_opencv_hdf=OFF \
-D BUILD_opencv_test_hdf=OFF \
-D BUILD_opencv_perf_imgproc=OFF \
-D BUILD_opencv_test_imgproc=OFF \
-D BUILD_opencv_intensity_transform=OFF \
-D BUILD_opencv_test_intensity_transform=OFF \
-D BUILD_opencv_ml=OFF \
-D BUILD_opencv_test_ml=OFF \
-D BUILD_opencv_phase_unwrapping=OFF \
-D BUILD_opencv_test_phase_unwrapping=OFF \
-D BUILD_opencv_plot=OFF \
-D BUILD_opencv_quality=OFF \
-D BUILD_opencv_test_quality=OFF \
-D BUILD_opencv_reg=OFF \
-D BUILD_opencv_test_reg=OFF \
-D BUILD_opencv_perf_reg=OFF \
-D BUILD_opencv_surface_matching=OFF \
-D BUILD_opencv_alphamat=OFF \
-D BUILD_opencv_cudafilters=OFF \
-D BUILD_opencv_test_cudafilters=OFF \
-D BUILD_opencv_perf_cudafilters=OFF \
-D BUILD_opencv_cudaimgproc=OFF \
-D BUILD_opencv_test_cudaimgproc=OFF \
-D BUILD_opencv_perf_cudaimgproc=OFF \
-D BUILD_opencv_cudawarping=OFF \
-D BUILD_opencv_test_cudawarping=OFF \
-D BUILD_opencv_perf_cudawarping=OFF \
-D BUILD_opencv_dnn=OFF \
-D BUILD_opencv_perf_dnn=OFF \
-D BUILD_opencv_test_dnn=OFF \
-D BUILD_opencv_dnn_superres=OFF \
-D BUILD_opencv_test_dnn_superres=OFF \
-D BUILD_opencv_perf_dnn_superres=OFF \
-D BUILD_opencv_features2d=ON \
-D BUILD_opencv_perf_features2d=OFF \
-D BUILD_opencv_test_features2d=OFF \
-D BUILD_opencv_freetype=OFF \
-D BUILD_opencv_fuzzy=OFF \
-D BUILD_opencv_test_fuzzy=OFF \
-D BUILD_opencv_hfs=OFF \
-D BUILD_opencv_img_hash=OFF \
-D BUILD_opencv_test_img_hash=OFF \
-D BUILD_opencv_perf_imgcodecs=OFF \
-D BUILD_opencv_test_imgcodecs=OFF \
-D BUILD_opencv_line_descriptor=OFF \
-D BUILD_opencv_test_line_descriptor=OFF \
-D BUILD_opencv_perf_line_descriptor=OFF \
-D BUILD_opencv_photo=OFF \
-D BUILD_opencv_test_photo=OFF \
-D BUILD_opencv_perf_photo=OFF \
-D BUILD_opencv_saliency=OFF \
-D BUILD_opencv_test_saliency=OFF \
-D BUILD_opencv_text=OFF \
-D BUILD_opencv_test_text=OFF \
-D BUILD_opencv_test_videoio=OFF \
-D BUILD_opencv_perf_videoio=OFF \
-D BUILD_opencv_xphoto=OFF \
-D BUILD_opencv_test_xphoto=OFF \
-D BUILD_opencv_perf_xphoto=OFF \
-D BUILD_opencv_calib3d=ON \
-D BUILD_opencv_perf_calib3d=OFF \
-D BUILD_opencv_test_calib3d=OFF \
-D BUILD_opencv_cudafeatures2d=OFF \
-D BUILD_opencv_test_cudafeatures2d=OFF \
-D BUILD_opencv_perf_cudafeatures2d=OFF \
-D BUILD_opencv_cudastereo=OFF \
-D BUILD_opencv_test_cudastereo=OFF \
-D BUILD_opencv_perf_cudastereo=OFF \
-D BUILD_opencv_datasets=OFF \
-D BUILD_opencv_test_highgui=OFF \
-D BUILD_opencv_objdetect=OFF \
-D BUILD_opencv_test_objdetect=OFF \
-D BUILD_opencv_perf_objdetect=OFF \
-D BUILD_opencv_rapid=OFF \
-D BUILD_opencv_test_rapid=OFF \
-D BUILD_opencv_rgbd=OFF \
-D BUILD_opencv_test_rgbd=OFF \
-D BUILD_opencv_shape=OFF \
-D BUILD_opencv_test_shape=OFF \
-D BUILD_opencv_structured_light=OFF \
-D BUILD_opencv_test_structured_light=OFF \
-D BUILD_opencv_video=ON \
-D BUILD_opencv_perf_video=OFF \
-D BUILD_opencv_test_video=OFF \
-D BUILD_opencv_xfeatures2d=OFF \
-D BUILD_opencv_perf_xfeatures2d=OFF \
-D BUILD_opencv_test_xfeatures2d=OFF \
-D BUILD_opencv_ximgproc=ON \
-D BUILD_opencv_perf_ximgproc=OFF \
-D BUILD_opencv_test_ximgproc=OFF \
-D BUILD_opencv_xobjdetect=OFF \
-D BUILD_opencv_waldboost_detector=OFF \
-D BUILD_opencv_aruco=OFF \
-D BUILD_opencv_test_aruco=OFF \
-D BUILD_opencv_bgsegm=OFF \
-D BUILD_opencv_test_bgsegm=OFF \
-D BUILD_opencv_bioinspired=OFF \
-D BUILD_opencv_test_bioinspired=OFF \
-D BUILD_opencv_perf_bioinspired=OFF \
-D BUILD_opencv_ccalib=OFF \
-D BUILD_opencv_cudabgsegm=OFF \
-D BUILD_opencv_test_cudabgsegm=OFF \
-D BUILD_opencv_perf_cudabgsegm=OFF \
-D BUILD_opencv_cudalegacy=OFF \
-D BUILD_opencv_test_cudalegacy=OFF \
-D BUILD_opencv_perf_cudalegacy=OFF \
-D BUILD_opencv_cudaobjdetect=OFF \
-D BUILD_opencv_test_cudaobjdetect=OFF \
-D BUILD_opencv_perf_cudaobjdetect=OFF \
-D BUILD_opencv_dnn_objdetect=OFF \
-D BUILD_opencv_dpm=OFF \
-D BUILD_opencv_face=OFF \
-D BUILD_opencv_test_face=OFF \
-D BUILD_opencv_gapi=OFF \
-D BUILD_opencv_test_gapi=OFF \
-D BUILD_opencv_perf_gapi=OFF \
-D BUILD_opencv_optflow=OFF \
-D BUILD_opencv_test_optflow=OFF \
-D BUILD_opencv_perf_optflow=OFF \
-D BUILD_opencv_stitching=OFF \
-D BUILD_opencv_test_stitching=OFF \
-D BUILD_opencv_perf_stitching=OFF \
-D BUILD_opencv_tracking=OFF \
-D BUILD_opencv_test_tracking=OFF \
-D BUILD_opencv_perf_tracking=OFF \
-D BUILD_opencv_cudaoptflow=OFF \
-D BUILD_opencv_perf_cudaoptflow=OFF \
-D BUILD_opencv_test_cudaoptflow=OFF \
-D BUILD_opencv_stereo=OFF \
-D BUILD_opencv_test_stereo=OFF \
-D BUILD_opencv_perf_stereo=OFF \
-D BUILD_opencv_superres=OFF \
-D BUILD_opencv_test_superres=OFF \
-D BUILD_opencv_perf_superres=OFF \
-D BUILD_opencv_videostab=OFF \
-D BUILD_opencv_test_videostab=OFF \
-D BUILD_gen_opencv_python_source=OFF \
-D BUILD_opencv_python2=OFF \
-D BUILD_opencv_python3=OFF \
-D BUILD_opencv_annotation=OFF \
-D BUILD_opencv_visualisation=OFF \
-D BUILD_opencv_interactive-calibration=OFF \
-D BUILD_opencv_versio=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=OFF \
    -D BUILD_EXAMPLES=OFF ..

make -j4
sudo make install
sudo ldconfig
pkg-config --modversion opencv

rm -rf ~/opencv ~/opencv_contrib
rm ~/opencv.zip ~/opencv_contrib.zip
