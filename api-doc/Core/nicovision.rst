nicovision
**********

The **nicovision** package contains classes to control the audio interface of the NICO.

VideoRecorder
#############

The VideoRecorder class manages the caption of videos from the cameras of the NICO robot.
To choose between different video codecs one can use the VideoCodec class

.. automodule:: nicovision.VideoRecorder
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

ImageRecorder
#############

The ImageRecorder class enables the capturing of single images from a camera.

.. automodule:: nicovision.ImageRecorder
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

MultiCamRecorder
################

The MultiCamRecorder class enables synchronized capturing of single images from multiple cameras at once.

.. automodule:: nicovision.MultiCamRecorder
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

PyrepRecorder
#############

The PyrepRecorder class allows recording images and videos from CoppeliaSim vision sensors using pyrep.

.. automodule:: nicovision.PyrepRecorder
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

CameraCalibrator
################

CameraCalibrator allows to calibrate the cameras with a chessboard pattern

.. automodule:: nicovision.CameraCalibrator
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

Display
#######

The Display class displays multiple cameras in one window

.. automodule:: nicovision.Display
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

Helper classes
##############

Barrier
=======

The Barrier class provides a Python 2 compatible barrier for thread synchronization

.. automodule:: nicovision.Barrier
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

Colorspace
==========

The Colorspace class represents different colorspaces.

.. automodule:: nicovision.Colorspace
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

ImageWriter
===========

The ImageWriter class saves image with multiple worker threads to allow capturing high resolution images at a high frequency

.. automodule:: nicovision.ImageWriter
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

PyrepDevice
===========

The PyrepDevice class handles communication with CoppeliaSim Vision Sensors via Pyrep.
It allows registering callback functions which are executed at each simulation step.

.. automodule:: nicovision.PyrepDevice
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

VideoDevice
===========

The VideoDevice class handles low-level communication with the video capture devices.

.. automodule:: nicovision.VideoDevice
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:
