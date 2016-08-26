#!/usr/bin/env python

import time

from nicovision import VideoRecorder

recording_time = 15

print('Starting a recording for %i seconds' % recording_time)
r = VideoRecorder.VideoRecorder(videoformat=VideoRecorder.VideoCodec.DIVX)
r.startRecording('./', 'VideoRecorderExample')
print('Now recording...')
time.sleep(recording_time)
r.stopRecording()
print('Recording finished')
