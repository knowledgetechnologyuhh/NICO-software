rosnico - Vision
****************

The Vision class allows the access to the cameras through ROS.

To start the interface run:

.. code-block:: bash
   
   rosrun nicoros Vision.py

Many options are available to change the behaviour. For a list of them run:

.. code-block:: bash
   
   rosrun nicoros Vision.py -h

Exposed topics
##############
The following ROS topics are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/vision):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/videoStream                            | sensor_msgs/Image                              | Video stream of the camera                                                            |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Class documentation
###################

.. automodule:: Vision
    :members:
    :undoc-members:
    :show-inheritance: