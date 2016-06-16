#!/bin/bash
echo "Uploading code to arduino Mega"
cd ~/catkin_ws/build
make goto_box_firmware_wheelchair_mega_wheelchair-upload
