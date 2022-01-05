#!/bin/bash

MY_PROMPT='$ '

echo 
echo "Creating Udev rule for PAL camera";

echo "KERNEL==\"video*\", SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"2560\", ATTR{index}==\"0\", ATTRS{manufacturer}==\"e-con systems\", SYMLINK+=\"pal5\"" 1>/etc/udev/rules.d/pal5.rules


echo "Udev rule created for PAL Camera";
echo 
echo "[INFO] PLEASE REBOOT THE SYSTEM FOR CHANGES TO TAKE EFFECT"
exit 0
