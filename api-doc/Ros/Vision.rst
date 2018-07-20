nicoros - Vision
****************

The Vision class allows the access to the cameras through ROS.

To start the interface run one of the preset launch files:

.. code-block:: bash

  # old cameras
  roslaunch nicoros nicoros_vision_Logitech_C905.launch
  # new cameras
  roslaunch nicoros nicoros_vision_See3CAM_CU135.launch

or start the node manually using:

.. code-block:: bash

   rosrun nicoros Vision.py

Many options are available to change the behaviour. For a list of them run:

.. code-block:: bash

   rosrun nicoros Vision.py -h

most of these also apply to the launch files

Exposed topics
##############
The following ROS topics are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/vision):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/left                                   | sensor_msgs/Image                              | Video stream of the left camera                                                       |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/right                                  | sensor_msgs/Image                              | Video stream of the right camera                                                      |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Exposed services
################

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/vision):

+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| ROS service name                               | Service type                                   | Short description                                                          |
+================================================+================================================+============================================================================+
| $PREFIX/setZoom                                | nicomsg/SetIntValue                            | Sets the zoom of the camera (if any) and returns whether successful        |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/setPan                                 | nicomsg/SetIntValue                            | Sets the pan of the camera (if any) and returns whether successful         |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/setTilt                                | nicomsg/SetIntValue                            | Sets the tilt of the camera (if any) and returns whether successful        |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+



Class documentation
###################

.. automodule:: Vision
    :members:
    :undoc-members:
    :show-inheritance:

Stereo Vision
#############
.. note::

    The Vision module has been updated and is now capable to stream multiple cameras at once. This section is therefore obsolete.

Both cameras can be launched simultaneously with the stereoVision.launch file. It requires the libuvc_camera package (http://wiki.ros.org/libuvc_camera).
To execute it run:

.. code-block:: bash

   roslaunch nicoros stereoVision.launch serialL:="2DE7B460" serialR:="17E79161"

The arguments serialL and serialR are used to address the left and right camera by their serial. To get the serials of all connected cameras run:

.. code-block:: bash

   lsusb -v -d 046d:080a | grep -i serial

To change the camera settings edit the parameter values in the camera.launch file or add new parameters to change options that are
currently not set. The full list of possible parameters can be found here: http://wiki.ros.org/libuvc_camera#Parameters

Keep in mind however that due to the fact that there is twice as much data, high resolutions are only possible with lower frame rates.
The following table shows the highest possible frame rate for the resolutions supported by the webcam (last checked on 23.12.2016):

+--------------------------------+----------------+
| Resolution                     | Maximum fps    |
+================================+================+
| 1600x1200, 1600x1000, 1600x904 | 0              |
+--------------------------------+----------------+
| 1280x800, 1280x720, 960x720    | 5              |
+--------------------------------+----------------+
|  864x480, 800x600, 800x504     | 10             |
+--------------------------------+----------------+
|  800x456, 768x480, 640x480     | 15             |
+--------------------------------+----------------+
|  640x400, 640x360              | 20             |
+--------------------------------+----------------+
|  352x288, 320x240 and lower    | 30             |
+--------------------------------+----------------+

The cameras use the /stereo/left and /stereo/right namespaces for their topics and services. The published topics and services are listed
here: http://wiki.ros.org/libuvc_camera#camera_node
