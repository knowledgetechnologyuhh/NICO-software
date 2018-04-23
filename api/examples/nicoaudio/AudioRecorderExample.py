#!/usr/bin/env python

from nicoaudio import AudioRecorder
import time

recorder = AudioRecorder.AudioRecorder()


recorder.startMicrophonesRecording(filename="AudioRecorderExample.wav", type="wav", # type argument is not implemented yet, the output file is always wav
                                   samplerate=44100, channels=(True,True)) # channels can be used to disable the left or right channel in the output file
print("Start recording")


time.sleep(5.0) # record for 5 seconds

recorder.stopMicrophonesRecording()
print("Recording stopped")
