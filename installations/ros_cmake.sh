cat /home/$SUDO_USER/DreamVu/PAL-USB/dreamvu_pal_camera/cmake_template/header.txt > /home/$SUDO_USER/DreamVu/PAL-USB/dreamvu_pal_camera/CMakeLists.txt
echo "set(PAL_INCLUDE_DIR" /home/$SUDO_USER/DreamVu/PAL-USB/include ")" >> /home/$SUDO_USER/DreamVu/PAL-USB/dreamvu_pal_camera/CMakeLists.txt
echo "set(PAL_LIBRARY" /home/$SUDO_USER/DreamVu/PAL-USB/lib/libPAL_PAL.so  /home/$SUDO_USER/DreamVu/PAL-USB/lib/libPAL_CAMERA_PAL.so /home/$SUDO_USER/DreamVu/PAL-USB/lib/libPAL_Track_PAL.so ")" >> /home/$SUDO_USER/DreamVu/PAL-USB/dreamvu_pal_camera/CMakeLists.txt
cat /home/$SUDO_USER/DreamVu/PAL-USB/dreamvu_pal_camera/cmake_template/footer.txt >> /home/$SUDO_USER/DreamVu/PAL-USB/dreamvu_pal_camera/CMakeLists.txt
