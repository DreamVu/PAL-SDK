#!/bin/bash

python3 -m venv dreamvu_ws
source ./dreamvu_ws/bin/activate
pip install --upgrade setuptools pip

pip install --upgrade google-api-python-client google-auth-httplib2 google-auth-oauthlib

pip install Cython
pip install numpy==1.19.3
pip install pyyaml imageio opencv-python-headless 

pip install pyparsing==3.0.8

echo "export LD_LIBRARY_PATH=$"LD_LIBRARY_PATH":/usr/local/lib" >> ~/.bashrc 
echo "export LD_LIBRARY_PATH=$"LD_LIBRARY_PATH":/opt/nvidia/vpi1/lib64" >> ~/.bashrc 
echo "source `pwd`/dreamvu_ws/bin/activate" >> ~/.bashrc
