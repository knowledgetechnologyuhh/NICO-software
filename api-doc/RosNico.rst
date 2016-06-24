RosNico
*******

The RosNico class allows control over the NICO robot by communicating through ros.

To start the interface run:

.. code-block:: bash
   
   rosrun high_nico RosNico.py

Many options are available to change the behaviour of RosNico. For a list of them run:

.. code-block:: bash
   
   rosrun high_nico RosNico.py -h

Exposed topics
##############
The following ROS topics are exposed by RosNico - where $PREFIX stands for the prefix of the topics/services (Default: /nico):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/changeAngle                            | high_nico/sff                                  | Changes the angle of a single joint. Parameters:                                      |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. Angle                                                                             |
|                                                |                                                |  3. Fraction of max speed                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/closeHand                              | high_nico/s                                    | Closes the hand. Parameters:                                                          |
|                                                |                                                |  1. "RHand" or "LHand"                                                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/disableForceControl                    | high_nico/empty                                | Disables force control for all joints                                                 |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/disableForceControlSingleJoint         | high_nico/s                                    | Disables force control for a single joint. Parameters:                                |
|                                                |                                                |  1. Name of joint                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/enableForceControl                     | high_nico/i                                    | Enables force control on all supported joints. Parameters:                            |
|                                                |                                                |  1. Goal force                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/enableForceControlSingleJoint          | high_nico/si                                   | Enables force control on a single joint. Parameters:                                  |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. Goal force                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/openHand                               | high_nico/s                                    | Opems the hand. Parameters:                                                           |
|                                                |                                                |  1. "RHand" or "LHand"                                                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setAngle                               | high_nico/sff                                  | Sets the angle of a single joint. Parameters:                                         |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. angle                                                                             |
|                                                |                                                |  3. Fraction of max speed                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setMaximumSpeed                        | high_nico/sff                                  | Sets the maximum allowed speed (in fraction of maximum possible speed Parameters:     |
|                                                |                                                |  1. Maximum speed                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Exposed services
################

The following ROS services are exposed by RosNico - where $PREFIX stands for the prefix of the topics/services (Default: /nico):

+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| ROS service name                               | Service type                                   | Short description                                                          |
+================================================+================================================+============================================================================+
| $PREFIX/getAngle                               | high_nico/get_value                            | Returns the angle of a single joint                                        | 
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getAngleLowerLimit                     | high_nico/get_value                            | Returns the lower limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getAngleUpperLimit                     | high_nico/get_value                            | Returns the upper limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getCurrent                             | high_nico/get_value                            | Returns the present currency of a single joint                             |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getJointNames                          | high_nico/get_names                            | Returns a list with the names of all joints                                |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getSensorNames                         | high_nico/get_names                            | Returns a list with the names of all sensors                               |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getTemperature                         | high_nico/get_value                            | Returns the present temperature of a single joint                          |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getTorqueLimit                         | high_nico/get_value                            | Returns the torque limit of a single joint                                 |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+

Class documentation
###################

.. automodule:: RosNico
    :members:
    :undoc-members:
    :show-inheritance:
