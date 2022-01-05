sudo cp dualenet.onnx /usr/local/bin
sudo cp -r data/ /usr/local/bin
sudo cp depth*.bin /usr/local/bin
sudo chown -R $USER:$USER /usr/local/bin/depth*.bin /usr/local/bin/dualenet.onnx /usr/local/bin/data/*/*.png

sudo ldconfig
