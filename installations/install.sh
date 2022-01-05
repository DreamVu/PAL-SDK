chmod +x ./dependencies.sh
sudo ./dependencies.sh

chmod +x ./setup_env.sh
./setup_env.sh

sudo cp ./bin_files/depth.bin /usr/local/bin/depth.bin

python3 test_py_installations.py

chmod +x ./PAL_udev.sh
sudo ./PAL_udev.sh

