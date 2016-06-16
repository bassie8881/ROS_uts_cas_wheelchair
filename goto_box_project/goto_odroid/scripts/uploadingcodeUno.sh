#!/bin/bash
echo "Uploading code to arduino Uno"
cd ~/catkin_ws/build
make goto_odroid_firmware_extensional_module_boxinfo-upload
