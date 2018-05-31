#!/bin/bash
./del.sh
rm ./audio/*.wav
mkdir ./audio
mkdir ./recorded_images
mkdir ./recorded_images/camera1
mkdir ./recorded_images/camera2
python multimodal_writer_vision_joints_sound.py 1920 1080 30 2
ls -1 ./recorded_images/camera1/ | wc -l
wc -l ./left_cam_synced_data.csv
ls -1 ./recorded_images/camera2/ | wc -l
wc -l ./right_cam_synced_data.csv
sox ./audio/*.wav -n stat 2>&1 | sed -n 's#^Length (seconds):[^0-9]*\([0-9.]*\)$#\1#p'

