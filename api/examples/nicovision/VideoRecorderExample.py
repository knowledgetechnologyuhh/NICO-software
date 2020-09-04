#!/usr/bin/env python

import time

from nicovision import VideoRecorder

recording_time = 15

print('Starting a recording for %i seconds' % recording_time)
r = VideoRecorder.VideoRecorder(device=VideoRecorder.get_devices()[0],
                                width=1920, height=1080, framerate=30,
                                videoformat=VideoRecorder.VideoCodec.XVID)

r.start_recording('./', 'VideoRecorderExample')
print('Now recording...')
time.sleep(recording_time)
r.stop_recording()
print('Recording finished')
