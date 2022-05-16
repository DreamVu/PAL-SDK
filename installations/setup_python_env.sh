#!/bin/bash
#sudo rm -rf dreamvu_ws

python3 -m venv dreamvu_ws
source ./dreamvu_ws/bin/activate
pip install --upgrade setuptools pip

pip install --upgrade google-api-python-client google-auth-httplib2 google-auth-oauthlib
#pip install -U grpcio absl-py 

#torch installation
#wget https://nvidia.box.com/shared/static/wa34qwrwtk9njtyarwt5nvo6imenfy26.whl -O torch-1.7.0-cp36-cp36m-linux_aarch64.whl

pip install Cython
pip install numpy==1.19.3
pip install pyyaml imageio opencv-python-headless 
#pip install torch-1.7.0-cp36-cp36m-linux_aarch64.whl

#torchvision installation
# git clone --branch v0.8.1 https://github.com/pytorch/vision torchvision   
# cd torchvision
# export BUILD_VERSION=0.8.1
# python3 setup.py install     
# cd ../ 
# pip install geffnet
pip install pyparsing==3.0.8

echo "export LD_LIBRARY_PATH=$"LD_LIBRARY_PATH":`pwd`/../lib" >> ~/.bashrc 
echo "export LD_LIBRARY_PATH=$"LD_LIBRARY_PATH":/opt/nvidia/vpi1/lib64" >> ~/.bashrc 
echo "source `pwd`/dreamvu_ws/bin/activate" >> ~/.bashrc
