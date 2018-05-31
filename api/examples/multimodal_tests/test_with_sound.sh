#!/bin/bash
./del.sh
rm ./audio/*.wav
mkdir ./audio
mkdir ./recorded_images
mkdir ./recorded_images/camera1
mkdir ./recorded_images/camera2
python multimodal_writer_vision_joints_sound.py 1920 1080 30 2
RCP1=$(ls -1 ./recorded_images/camera1/ | wc -l)
DL1=$(wc -l ./left_cam_synced_data.csv | awk '{print $1}' )
DL=$(expr $DL1 - 1 )
printf "\n\nWe ran a test for 5s at 30fps. So we exspect at about 150 pictures, 150 datalines and 5s of sound recording.\n\n"
printf "\nPics from cam1: %s Datalines: %s\n" "$RCP1" "$DL"
RCP2=$(ls -1 ./recorded_images/camera2/ | wc -l)
DL2=$(wc -l ./right_cam_synced_data.csv | awk '{print $1}' )
DL=$(expr $DL2 - 1 )
printf "\nPics from cam2: %s Datalines: %s\n" "$RCP2" "$DL"
SLEN=$(sox ./audio/*.wav -n stat 2>&1 | sed -n 's#^Length (seconds):[^0-9]*\([0-9.]*\)$#\1#p')
printf "\nLength of sound recording: %s seconds\n\n" "$SLEN"



