#!/usr/bin/env python

import logging
import rospy
import nicomsg.msg
import nicomsg.srv
import audioop
from _nicoaudio_internal.record_sound import RecordSound

class AudioStream:
    def __init__(self):
      '''AudioStream allows to stream audiodata received from a microphone via ROS'''

      self._running = False

      # global audio recorder (initialized in startStream)
      self._recorder = None
      # init ROS
      rospy.init_node('audiostream', anonymous=True)
      # global publisher variables (initialized in startStream)
      self._publisherLeft = None     
      self._publisherRight= None 
      # global service variables (startStream initializes the other ones)
      self._getWidthSrv   = None
      self._getRateSrv    = None
      self._startSrv      = rospy.Service('nico/audiostream/startStream', nicomsg.srv.StartAudioStream, self._ROSPY_startStream)
      self._stopSrv       = None
      # actual streaming      
      sampleIndex = 0
      while not rospy.is_shutdown():
        if self._running and self._recorder.get_number_of_samples() > sampleIndex:
          sample = self._recorder.get_chunk(sampleIndex)
          sampleIndex += 1
          msg = nicomsg.msg.hs()
          msg.header.stamp = rospy.Time.now()
          if self._publisherLeft:
            msg.param1 = audioop.tomono(sample, self._recorder.get_sample_width(), 1, 0)
            self._publisherLeft.publish(msg)
          if self._publisherRight:
            msg.param1 = audioop.tomono(sample, self._recorder.get_sample_width(), 0, 1)
            self._publisherRight.publish(msg)
      if self._running:
        self.stopStream()

    def startStream(self, filename, samplerate, channels):
      '''
      Starts the audiostream, the source is saved after the stream ends
      :param filename: name for the output file
      :type filename: str
      :param channels: channels that should be streamed (left,right)
      :type channels: tuple(bool, bool)
      :return: success      
      :rtype: bool
      '''
      if self._running:    
        return False
      self._filename = filename
      self._channels = channels
      self._recorder = RecordSound(rate = samplerate)
      self._recorder.start()
      self._running  = True
      # enable publishers depending on channels
      if channels[0]:
        self._publisherLeft = rospy.Publisher('nico/audiostream/left', nicomsg.msg.hs, queue_size = 10)
      if channels[1]:
        self._publisherRight = rospy.Publisher('nico/audiostream/right', nicomsg.msg.hs, queue_size = 10)
      # start runtime services
      self._getWidthSrv = rospy.Service('nico/audiostream/getSampleWidth', nicomsg.srv.GetIntValue, self._ROSPY_getSampleWidth)
      self._getRateSrv  = rospy.Service('nico/audiostream/getFrameRate', nicomsg.srv.GetIntValue, self._ROSPY_getFrameRate)
      self._stopSrv     = rospy.Service('nico/audiostream/stopStream', nicomsg.srv.StopAudioStream, self._ROSPY_stopStream)
      # disable start service
      self._startSrv.shutdown()
      return True
      
    def stopStream(self):
      '''Stops the audiostream and saves the file
      :return: success      
      :rtype: bool   
      '''
      if not self._running:
        return False
      self._running = False
      self._recorder.stop()
      self._recorder.save(self._filename, self._channels)
      # disable publishers
      if self._publisherLeft:
        self._publisherLeft.unregister()
      if self._publisherRight:
        self._publisherRight.unregister()
      # shutdown runtime services
      self._getWidthSrv.shutdown()
      self._getRateSrv.shutdown()
      self._stopSrv.shutdown()
      # reenable start service
      self._startSrv = rospy.Service('nico/audiostream/startStream', nicomsg.srv.StartAudioStream, self._ROSPY_startStream)
      return True
    
#----------------------------------------Service Callbacks----------------------------------------#
    
    def _ROSPY_startStream(self,msg):
      '''
      ROS service handle to start the audio stream
      :param message: ROS message
      :type message: nicomsg.srv.StartAudioStream
      :return: success      
      :rtype: bool     
      '''
      return self.startStream(msg.filename,msg.samplerate,(msg.left, msg.right))

    def _ROSPY_stopStream(self,_):
      '''
      ROS service handle to stop the audio stream
      :param message: ROS message
      :type message: nicomsg.srv.StopAudioStream
      :return: success      
      :rtype: bool      
      '''
      return self.stopStream()

    def _ROSPY_getSampleWidth(self,_):
        '''
        ROS service handle to get the sample width of the stream (needed for wave)
        :return: sample width of the audio stream      
        :rtype: int        
        '''
        if self._recorder:
          return self._recorder.get_sample_width()
        return 0

    def _ROSPY_getFrameRate(self,_):
      '''
      ROS service handle to get the sampling frequency of the stream (needed for wave)
      :return: sampling rate of audio stream      
      :rtype: int
      '''
      if self._recorder:          
        return self._recorder._rate
      return 0

    def _ROSPY_getChannelStates(self,_):
      '''
      ROS Service to see which channels are currently streamed
      :return: streaming status of the two channels
      :rtype: tuple(bool left, bool right)
      '''
      if self._recorder:          
        return self._channels
      return False, False       

if __name__ == '__main__':
  stream = AudioStream()
