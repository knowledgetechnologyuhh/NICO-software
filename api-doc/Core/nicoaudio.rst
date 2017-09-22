nicoaudio
*********

The **nicoaudio** package contains classes to control the audio interface of the NICO.

AudioPlayer
###########

The AudioPlayer class manages the playback of audio through the speakers on the NICO robot.

.. automodule:: nicoaudio.AudioPlayer
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:


AudioRecorder
#############

The AudioRecorder class manages the caption of audio from the microphones on the NICO robot.

.. automodule:: nicoaudio.AudioRecorder
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

AudioStream
###########

The AudioStream class allows to send the audio data through ROS.

To start the interface run:

.. code-block:: bash

   rosrun nicoaudio AudioStream.py

Exposed topics
==============

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/audiostream):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/left                                   | nicomsg/hs                                     | latest pyaudio chunk recorded by the left microphone with timestamp                   |
|                                                |                                                | (only visible when enabled and the stream is running)                                 |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/right                                  | nicomsg/hs                                     | latest pyaudio chunk recorded by the right microphone with timestamp                  |
|                                                |                                                | (only visible when enabled and the stream is running)                                 |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Exposed services
================

The following ROS topics are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/audiostream):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/startStream                            | nicomsg/StartAudioStream                       | Starts the audiostream. (only visible if the stream is not running)                   |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |  1. filename of the outputfile (stream is saved after termination)                    |
|                                                |                                                |  2. samplerate of the recording                                                       |
|                                                |                                                |  3. enable streaming of left audiochannel?                                            |
|                                                |                                                |  4. enable streaming of right audiochannel?                                           |
|                                                |                                                |                                                                                       |
|                                                |                                                | Returns True if successful.                                                           |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/stopStream                             | nicomsg/StopAudioStream                        | Stops the audiostream. (only visible while stream is running)                         |
|                                                |                                                |                                                                                       |
|                                                |                                                | Returns True if successful.                                                           |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/getFrameRate                           | nicomsg/GetIntValue                            | Returns the samplerate at which the stream is recorded. (only visible while running)  |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/getSampleWidth                         | nicomsg/GetIntValue                            | Returns the samplewidth at which the stream is recorded. (only visible while running) |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+


.. automodule:: nicoaudio.AudioStream
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:
