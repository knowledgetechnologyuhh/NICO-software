nicoros - AudioPlayer
*********************

The AudioPlayer class allows to load and play audio files.

To start the interface run:

.. code-block:: bash

   rosrun nicoros AudioPlayer.py

Exposed topics
==============

The following ROS topics are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: nico/audioplayer):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/play                                   | nicomsg/sf                                     | Starts playback of an audio segment. Parameters:                                      |
|                                                |                                                |  - id: id of the audio assigned by the player                                         |
|                                                |                                                |  - volume: percentage of the volume (between 0 and 1)                                 |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/pause                                  | nicomsg/s                                      | Pauses the playback of an audio segment. Parameters:                                  |
|                                                |                                                | - id: id of the audio assigned by the player                                          |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/resume                                 | nicomsg/s                                      | Resumes the playback of an audio segment. Parameters:                                 |
|                                                |                                                | - id: id of the audio assigned by the player                                          |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/adjust_pitch                           | nicomsg/sf                                     | Adjusts pitch of an audio segment. Parameters:                                        |
|                                                |                                                |  1. id of the audio assigned by the player                                            |
|                                                |                                                |  2. pitch in octaves                                                                  |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/adjust_speed                           | nicomsg/sf                                     | Adjusts speed of an audio segment. Parameters:                                        |
|                                                |                                                |  1. id of the audio assigned by the player                                            |
|                                                |                                                |  2. factor by which to change speed                                                   |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Exposed services
================

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/audioplayer):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/load_file                              | nicomsg/LoadAudio                              | Loads (part) of the given audiofile                                                   |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |  - filename: file to load                                                             |
|                                                |                                                |  - start_seconds: point from which on to load the audio                               |
|                                                |                                                |  - duration_seconds: duration of the segment to load (loads full file if 0)           |
|                                                |                                                |                                                                                       |
|                                                |                                                | Returns id assigned by the player                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/get_position                           | nicomsg/GetValue                               | Returns current playback position of the given audio. Parameters:                     |
|                                                |                                                |  1. id of the audio assigned by the player                                            |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/get_duration                           | nicomsg/GetValue                               | Returns duration of the given audio. Parameters:                                      |
|                                                |                                                |  1. id of the audio assigned by the player                                            |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/get_filename                           | nicomsg/GetFilename                            | Returns filename of the given audio. Parameters:                                      |
|                                                |                                                |  1. id of the audio assigned by the player                                            |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/get_audio_ids                          | nicomsg/GetAudioIDs                            | Returns ids of all loaded audio segments                                              |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Class documentation
===================

.. automodule:: AudioPlayer
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:
