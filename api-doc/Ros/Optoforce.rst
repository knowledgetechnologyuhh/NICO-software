nicoros - Optoforce
*********************

The Optoforce class allows to send the Optoforce data through ROS.

To start the interface run:

.. code-block:: bash

   rosrun nicoros Optoforce.py

Exposed topics
==============

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/optoforce/$SERIAL where $SERIAL is the serial number of the sensor):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/raw                                    | nicomsg/iii                                    | raw sensor data                                                                       |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/newton                                 | nicomsg/fff                                    | sensor data in newton (make sure the serial is correct as conversion is based on it)  |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Class documentation
===================

.. automodule:: Optoforce
    :members:
    :undoc-members:
    :show-inheritance:
