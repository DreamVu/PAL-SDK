#!/bin/bash
chmod +x ./*.sh

./uninstall.sh

cd activation

chmod +x run ./*.sh

sudo cp *.json /usr/local/bin/data
sudo chown -R $SUDO_USER:$SUDO_USER /usr/local/bin/data/*.json

cd ..

./dependencies.sh

./ros_cmake.sh


./setup_python_env.sh

cd bin_files

chmod +x setup.sh
./setup.sh

echo "Status"
echo $1


if [[ $1 == "Y" || $1 == "y" ]]; 
then 
	chmod +x build_engines.sh 
	if [ $# -eq 2 ];
	then
		./build_engines.sh $2
	else
		./build_engines.sh 3500
	fi   
else
	echo "[INFO] Skipping Rebuilding Engines"
fi


cd ..

source dreamvu_ws/bin/activate
python test_py_installations.py

cd activation
./activation.sh

cd ..


./timeout_patch.sh

./PAL_udev.sh
