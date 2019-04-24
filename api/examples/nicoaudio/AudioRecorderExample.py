#!/usr/bin/env python

import logging
import time

from nicoaudio import AudioRecorder

logging.basicConfig(level=logging.INFO)


recorder = AudioRecorder.AudioRecorder()


recorder.startMicrophonesRecording(
    filename="AudioRecorderExample.wav",
    type="wav",  # has no effect (yet), the output file is always wav
    samplerate=44100,
    channels=(True, True))  # disables channel in output file if set to false
print("Start recording")


time.sleep(5.0)  # record for 5 seconds

recorder.stopMicrophonesRecording()
print("Recording stopped")
