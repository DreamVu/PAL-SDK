sudo cp -r data/ /usr/local/bin
# data contains all trt files and er and lut folder
sudo chown -R $SUDO_USER:$SUDO_USER /usr/local/bin/data/*.trt /usr/local/bin/data/*/*.png /usr/local/bin/data/unit.txt /usr/local/bin/data/*.json 
sudo mkdir -p /usr/src/tensorrt/bin/common/
sudo cp logger.o /usr/src/tensorrt/bin/common/
sudo chown -R  $SUDO_USER:$SUDO_USER /usr/src/tensorrt/
