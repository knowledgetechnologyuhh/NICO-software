# script for converting all images into a video

#usage: sh convertVision.sh path_to_individual_recording
#example: sh convertVision.sh push/48
#avoid spaces in the file path!
echo *** convert all images into a video ***

inp_vid_w=1920
inp_vid_h=1080 
out_vid_w_crop_fov=1024
out_vid_h_crop_fov=768 
out_vid_w_crop_table=800
out_vid_h_crop_table=600 
vid_fps=30
vid_codec=libx264
vid_loss=0
cam_folder=camsubset 
vid_enc_preset=ultrafast
#vid_enc_preset=veryslow

out_ftype=mp4
out_fname_full=cam_vision_full
out_fname_crop_table=cam_vision_table
out_fname_crop_fov=cam_vision_fov
out_rarname=cameras.rar

sample_path=$1

#left camera (camera1)
#the left camera is also automatically rotated (180°)
cam_path=camera1
cam_label=left_

cat $sample_path/$cam_path/picture*.png | ffmpeg -f image2pipe -framerate $vid_fps -i - -vf "transpose=1,transpose=1" $sample_path/$cam_label$out_fname_full.$out_ftype -c:v $vid_codec -crf $vid_loss -preset $vid_enc_preset

cat $sample_path/$cam_path/picture*.png | ffmpeg -f image2pipe -framerate $vid_fps -i - -vf "transpose=1,transpose=1,crop=$out_vid_w_crop_fov:$out_vid_h_crop_fov:$((($inp_vid_w-$out_vid_w_crop_fov)/2)):$((($inp_vid_h-$out_vid_h_crop_fov)/2))" $sample_path/$cam_label$out_fname_crop_fov.$out_ftype -c:v $vid_codec -crf $vid_loss -preset $vid_enc_preset

cat $sample_path/$cam_path/picture*.png | ffmpeg -f image2pipe -framerate $vid_fps -i - -vf "transpose=1,transpose=1,crop=$out_vid_w_crop_table:$out_vid_h_crop_table:$((($inp_vid_w-$out_vid_w_crop_table)/2)):$((($inp_vid_h-$out_vid_h_crop_table)/2))" $sample_path/$cam_label$out_fname_crop_table.$out_ftype -c:v $vid_codec -crf $vid_loss -preset $vid_enc_preset

#right camera (camera2)
cam_path=camera2
cam_label=right_

cat $sample_path/$cam_path/picture*.png | ffmpeg -f image2pipe -framerate $vid_fps -i - -vf "transpose=1,transpose=1,transpose=1,transpose=1" $sample_path/$cam_label$out_fname_full.$out_ftype -c:v $vid_codec -crf $vid_loss -preset $vid_enc_preset

cat $sample_path/$cam_path/picture*.png | ffmpeg -f image2pipe -framerate $vid_fps -i - -vf "crop=$out_vid_w_crop_fov:$out_vid_h_crop_fov:$((($inp_vid_w-$out_vid_w_crop_fov)/2)):$((($inp_vid_h-$out_vid_h_crop_fov)/2))" $sample_path/$cam_label$out_fname_crop_fov.$out_ftype -c:v $vid_codec -crf $vid_loss -preset $vid_enc_preset

cat $sample_path/$cam_path/picture*.png | ffmpeg -f image2pipe -framerate $vid_fps -i - -vf "crop=$out_vid_w_crop_table:$out_vid_h_crop_table:$((($inp_vid_w-$out_vid_w_crop_table)/2)):$((($inp_vid_h-$out_vid_h_crop_table)/2))" $sample_path/$cam_label$out_fname_crop_table.$out_ftype -c:v $vid_codec -crf $vid_loss -preset $vid_enc_preset

#move all images into rar archives
rar m -m4 $sample_path/camera1/pngs.rar $sample_path/camera1/*.png

rar m -m4 $sample_path/camera2/pngs.rar $sample_path/camera2/*.png