#!/usr/bin/env python

from nicoaudio import AudioRecorder
import time
import logging

recorder = AudioRecorder.AudioRecorder()


recorder.startMicrophonesRecording(filename="AudioRecorderExample.wav", type="wav", # type argument is not implemented yet, the output file is always wav
                                   samplerate=44100, channels=(True,True)) # channels can be used to disable the left or right channel in the output file

time.sleep(5.0) # record for 5 seconds

recorder.stopMicrophonesRecording()
