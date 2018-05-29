#!/bin/bash
./del.sh
python ImageRecorderMultipleExample_multimodal_writer.py 1920 1080 30 2
ls -1 ./recorded_images/camera1/ | wc -l
wc -l ./left_cam_synced_data.csv
ls -1 ./recorded_images/camera2/ | wc -l
wc -l ./right_cam_synced_data.csv
