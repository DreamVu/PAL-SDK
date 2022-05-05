sed -i '/Firmware-v1.2-NX-J4.4/ s/^/#/' ~/.bashrc
sed -i '/PAL-Mini-Firmware/ s/^/#/' ~/.bashrc
sed -i '/PAL-Firmware/ s/^/#/' ~/.bashrc
sed -i '/PAL-Mini/ s/^/#/' ~/.bashrc
sed -i '/PAL-ODOA/ s/^/#/' ~/.bashrc
sed -i '/3D-Object-Detection-Tracking/ s/^/#/' ~/.bashrc
sed -i '/Obstacle-Detection/ s/^/#/' ~/.bashrc
sed -i '/PAL-USB s/^/#/' ~/.bashrc
sed -i '/SLAM s/^/#/' ~/.bashrc

echo "Uninstalled previous versions of Firmwares from bashrc"

