#!/bin/sh

unzip eigen-3.3.9.zip
cd eigen-3.3.9
mkdir build
cd build
cmake ..
sudo make install
cd ../../

sudo cp ../lib/libnvtvmr.so /usr/lib/aarch64-linux-gnu/tegra

echo "Installing GPIO dependencies"
git clone https://github.com/pjueon/JetsonGPIO
cd JetsonGPIO
mkdir build
cd build

cmake ..
make all
sudo make install
sudo ldconfig

sudo groupadd -f -r gpio
sudo usermod -a -G gpio $SUDO_USER

cd ../..
sudo cp JetsonGPIO/99-gpio.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger


echo '#!/bin/bash'  > ./autostart.sh
echo "export LD_LIBRARY_PATH=$"LD_LIBRARY_PATH":`pwd`/../lib" >> ./autostart.sh
echo "export LD_LIBRARY_PATH=$"LD_LIBRARY_PATH":/opt/nvidia/vpi-0.4/lib" >> ./autostart.sh
echo "source `pwd`/dreamvu_ws/bin/activate" >> ./autostart.sh

echo "cd " `pwd`/../code_samples >> ./autostart.sh
echo "./013_object_tracking_GPIO.out" >> ./autostart.sh
chmod +x autostart.sh



