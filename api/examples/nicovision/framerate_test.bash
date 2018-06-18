#!/bin/bash

# clean output directories
rm recorded_images/camera0/*.png
rm recorded_images/camera1/*.png
# resolution_x, resolution_y, framerate, number_of_cameras_to_use
python ImageRecorderMultipleExample.py 640 480 15 2
# count number of recorded images
echo "Number of images recorded by camera 0:"
find recorded_images/camera0 -type f | wc -l
echo "Number of images recorded by camera 1:"
find recorded_images/camera1 -type f | wc -l
