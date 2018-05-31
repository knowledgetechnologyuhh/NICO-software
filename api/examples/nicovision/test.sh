#!/bin/bash
./del.sh
python ImageRecorderMultipleExample_customized_writer.py 1920 1080 30 2
ls -1 ./recorded_images/camera1/ | wc -l
ls -1 ./recorded_images/camera2/ | wc -l
