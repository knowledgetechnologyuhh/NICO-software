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

+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                          |
+================================================+================================================+============================================================================+
| $PREFIX/changeAngle                            | high_nico/sff                                  | Changes the angle of a single joint. Parameters:                           |
|                                                |                                                |  1. Name of joint                                                          |
|                                                |                                                |  2. Angle                                                                  |
|                                                |                                                |  3. Fraction of max speed                                                  |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/closeHand                              | high_nico/s                                    | Closes the hand. Parameters:                                               |
|                                                |                                                |  1. "RHand" or "LHand"                                                     |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/disableForceControl                    | high_nico/empty                                | Disables force control for all joints                                      |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/disableForceControlSingleJoint         | high_nico/s                                    | Disables force control for a single joint. Parameters:                     |
|                                                |                                                |  1. Name of joint                                                          |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/enableForceControl                     | high_nico/i                                    | Enables force control on all supported joints. Parameters:                 |
|                                                |                                                |  1. Goal force                                                             |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/enableForceControlSingleJoint          | high_nico/si                                   | Enables force control on a single joint. Parameters:                       |
|                                                |                                                |  1. Name of joint                                                          |
|                                                |                                                |  2. Goal force                                                             |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/moveWrist                              | high_nico/sfff                                 | Moves the wrist. Parameters:                                               |
|                                                |                                                |  1. "RHand" or "LHand"                                                     |
|                                                |                                                |  2. Target x angle                                                         |
|                                                |                                                |  3. Target y angle                                                         |
|                                                |                                                |  4. Fraction of max speed                                                  |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/openHand                               | high_nico/s                                    | Opems the hand. Parameters:                                                |
|                                                |                                                |  1. "RHand" or "LHand"                                                     |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/setAngle                               | high_nico/sff                                  | Sets the angle of a single joint. Parameters:                              |
|                                                |                                                |  1. Name of joint                                                          |
|                                                |                                                |  2. angle                                                                  |
|                                                |                                                |  3. Fraction of max speed                                                  |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+


Exposed services
################

The following ROS services are exposed by RosNico - where $PREFIX stands for the prefix of the topics/services (Default: /nico):

+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| ROS service name                               | Service type                                   | Short description                                                          |
+================================================+================================================+============================================================================+
| /nico/getAngle                                 | high_nico/get_value                            | Returns the angle of a single joint                                        | 
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| /nico/getAngleLowerLimit                       | high_nico/get_value                            | Returns the lower limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| /nico/getAngleUpperLimit                       | high_nico/get_value                            | Returns the upper limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| /nico/getCurrent                               | high_nico/get_value                            | Returns the present currency of a single joint                             |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| /nico/getJointNames                            | high_nico/get_names                            | Returns a list with the names of all joints                                |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| /nico/getSensorNames                           | high_nico/get_names                            | Returns a list with the names of all sensors                               |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
|/nico/getTemperature                            | high_nico/get_value                            | Returns the present temperature of a single joint                          |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| /nico/getTorqueLimit                           | high_nico/get_value                            | Returns the torque limit of a single joint                                 |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+

Class documentation
###################

.. automodule:: RosNico
    :members:
    :undoc-members:
    :show-inheritance:
