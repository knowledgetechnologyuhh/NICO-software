#/bin/sh
# Merge the sound file and the video file of the multimodal recordings
# ./merge_sound_and_video.sh [videofile] [soundfile] [mergefile]

#ffmpeg has a problem with ":" in the filename, so make a temp copy of the file
cp $2 sound_temp.wav
ffmpeg -i $1 -i sound_temp.wav -c:v copy -c:a aac -strict experimental $3
rm sound_temp.wav
