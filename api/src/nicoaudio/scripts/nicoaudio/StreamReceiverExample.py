#!/usr/bin/env python

import wave
import rospy
import nicomsg.msg
import nicomsg.srv
import message_filters
import pyaudio
import audioop


class AudioReceiver:

    def __init__(self):
      '''Example class for receiving audiostreams and synchronizing the seperate audio channels'''

      self._frames = []
      # init ROS
      rospy.init_node('audioreceiver', anonymous=True)
      # wait for audiostream services
      rospy.wait_for_service('nico/audiostream/startStream')
      rospy.wait_for_service('nico/audiostream/stopStream')
      rospy.wait_for_service('nico/audiostream/getSampleWidth')
      rospy.wait_for_service('nico/audiostream/getFrameRate')
      # initialize audio attributes
      try:
          startStream = rospy.ServiceProxy('nico/audiostream/startStream', nicomsg.srv.StartAudioStream)
          resp0 = startStream('stream.wav', 44100, True, True)
          getSampleWidth = rospy.ServiceProxy('nico/audiostream/getSampleWidth', nicomsg.srv.GetIntValue)
          getFrameRate = rospy.ServiceProxy('nico/audiostream/getFrameRate', nicomsg.srv.GetIntValue)
          resp1 = getSampleWidth()          
          resp2 = getFrameRate()
          self._samplewidth = resp1.value
          self._framerate = resp2.value
      except rospy.ServiceException, e: "Service call failed: %s"%e      
      # subscriber setup
      left = message_filters.Subscriber("nico/audiostream/left", nicomsg.msg.hs)
      right = message_filters.Subscriber("nico/audiostream/right", nicomsg.msg.hs)
      # synchronize subscribers
      sync = message_filters.TimeSynchronizer([left, right], 10)
      sync.registerCallback(self.callback)

    def callback(self,left,right):
      """ receives and reunites both audio channels """
      lsample = audioop.tostereo(left.param1,self._samplewidth,1,0)
      rsample = audioop.tostereo(right.param1,self._samplewidth,0,1)
      self._frames.append(audioop.add(lsample, rsample, self._samplewidth))

    def save(self, filename='sound.wav'):
        """Save the frames to a WAV file.

        Args:
            filename: An optional string for the name of the sound file.
        """
        
        wf = wave.open(filename, 'wb')
        wf.setnchannels(2)
        wf.setsampwidth(self._samplewidth)
        wf.setframerate(self._framerate)
        wf.writeframes(b''.join(self._frames))
        wf.close()

if __name__ == '__main__':
    receiver = AudioReceiver()
    rospy.spin()
    receiver.save('sound2.wav')
