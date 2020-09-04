nicoros - TextToSpeech
**********************

The TextToSpeech class generates audio from text.

To start the interface run:

.. code-block:: bash

   rosrun nicoros TextToSpeech.py


Exposed services
================

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/audioplayer):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/say                                    | nicomsg/SayText                                | Loads (part) of the given audiofile                                                   |
|                                                |                                                |                                                                                       |
|                                                |                                                | Parameters:                                                                           |
|                                                |                                                |  - text: text to say                                                                  |
|                                                |                                                |  - language: language code (e.g. 'en-GB' or 'de')                                     |
|                                                |                                                |  - pitch: pitch in octaves (also affects speed)                                       |
|                                                |                                                |  - speed: factor by which to multiply speed                                           |
|                                                |                                                |  - blocking: whether the call should be held until audio is played                    |
|                                                |                                                |                                                                                       |
|                                                |                                                | Returns remaining duration of the audio                                               |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Class documentation
===================

.. automodule:: TextToSpeech
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:
