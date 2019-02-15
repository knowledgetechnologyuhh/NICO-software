#!/bin/sh
sox sub_.wav sub_t.wav trim 0 0.625
sox sub_t.wav -r 16000 sub_t_r.wav
sox sub_t_r.wav sub_t_r_l.wav remix 1
cp sub_t_r_l.wav /data/sound_classification/online_pred_sample.wav 
