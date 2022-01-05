#!/bin/bash
chmod +x ./*.sh
sudo ./dependencies.sh

./ros_cmake.sh

./setup_python_env.sh

cd camera_data
chmod +x setup_python_lib.sh
./setup_python_lib.sh

cd ..

source dreamvu_ws/bin/activate
python test_py_installations.py

sudo ./PAL_udev.sh

