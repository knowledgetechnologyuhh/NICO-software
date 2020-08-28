nicoros - OptoforceMultichannel
*******************************

The OptoforceMultichannel class allows to send the Optoforce data from a device
with multiple sensors through ROS.

To start the interface run:

.. code-block:: bash

   rosrun nicoros OptoforceMultichannel.py

Exposed topics
==============

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/optoforce/$SERIAL where $SERIAL is the serial number of the sensor)
and $CHANNEL is an identifier for the sensor:

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/raw/$CHANNEL                           | nicomsg/fff                                    | raw sensor data                                                                       |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/newton/$CHANNEL                        | nicomsg/fff                                    | sensor data in newton (only if conversion values known)                               |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Class documentation
===================

.. automodule:: OptoforceMultichannel
    :members:
    :undoc-members:
    :show-inheritance:
