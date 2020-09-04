nicoros - CapacitiveSensors
***************************

The CapacitiveSensors class publishes the sensor values of the capacitive pads
inside the NICO head through ROS.

To start the interface run (use -h to list optional arguments):

.. code-block:: bash

   rosrun nicoros CapacitiveSensors.py

Exposed topics
==============

The following ROS topics are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/CapacitiveSensors/):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/sensor_readings                        | nicomsg/ffff                                   | Sensor readings of each pad                                                           |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Class documentation
===================

.. automodule:: CapacitiveSensors
    :members:
    :undoc-members:
    :show-inheritance:
