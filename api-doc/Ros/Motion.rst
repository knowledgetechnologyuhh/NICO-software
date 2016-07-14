rosnico - Motion
****************

The Motion class allows control over the NICO robot by communicating through ROS.

To start the interface run:

.. code-block:: bash
   
   rosrun rosnico Motion.py

Many options are available to change the behaviour of RosNico. For a list of them run:

.. code-block:: bash
   
   rosrun rosnico Motion.py -h

Exposed topics
##############
The following ROS topics are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/motion):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/changeAngle                            | nico_msg/sff                                   | Changes the angle of a single joint. Parameters:                                      |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. Angle                                                                             |
|                                                |                                                |  3. Fraction of max speed                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/closeHand                              | nico_msg/s                                     | Closes the hand. Parameters:                                                          |
|                                                |                                                |  1. "RHand" or "LHand"                                                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/disableForceControl                    | nico_msg/empty                                 | Disables force control for all joints                                                 |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/disableForceControlSingleJoint         | nico_msg/s                                     | Disables force control for a single joint. Parameters:                                |
|                                                |                                                |  1. Name of joint                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/enableForceControl                     | nico_msg/i                                     | Enables force control on all supported joints. Parameters:                            |
|                                                |                                                |  1. Goal force                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/enableForceControlSingleJoint          | nico_msg/si                                    | Enables force control on a single joint. Parameters:                                  |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. Goal force                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/openHand                               | nico_msg/s                                     | Opems the hand. Parameters:                                                           |
|                                                |                                                |  1. "RHand" or "LHand"                                                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setAngle                               | nico_msg/sff                                   | Sets the angle of a single joint. Parameters:                                         |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. angle                                                                             |
|                                                |                                                |  3. Fraction of max speed                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setMaximumSpeed                        | nico_msg/f                                     | Sets the maximum allowed speed (in fraction of maximum possible speed). Parameters:   |
|                                                |                                                |  1. Maximum speed                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setPID                                 | nico_msg/sfff                                  | Sets the PID control of a single joint. Parameters:                                   |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. *p* (proportional band)                                                           |
|                                                |                                                |  3. *i* (integral action)                                                             |
|                                                |                                                |  4. *d* (derivative action)                                                           |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setStifftness                          | nico_msg/sf                                    | Sets the stifftness of a single joint. Parameters:                                    |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. stifftness                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Exposed services
################

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico):

+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| ROS service name                               | Service type                                   | Short description                                                          |
+================================================+================================================+============================================================================+
| $PREFIX/getAngle                               | nico_msg/GetValue                              | Returns the angle of a single joint                                        | 
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getAngleLowerLimit                     | nico_msg/GetValue                              | Returns the lower limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getAngleUpperLimit                     | nico_msg/GetValue                              | Returns the upper limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getCurrent                             | nico_msg/GetValue                              | Returns the present currency of a single joint                             |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getJointNames                          | nico_msg/GetNames                              | Returns a list with the names of all joints                                |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getPID                                 | nico_msg/GetPID                                | Returns the PID control of a single joint                                  |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getSensorNames                         | nico_msg/GetNames                              | Returns a list with the names of all sensors                               |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getStifftness                          | nico_msg/GetValue                              | Returns the stifftness of a single joint                                   |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getTemperature                         | nico_msg/GetValue                              | Returns the present temperature of a single joint                          |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getTorqueLimit                         | nico_msg/GetValue                              | Returns the torque limit of a single joint                                 |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+

Class documentation
###################

.. automodule:: Motion
    :members:
    :undoc-members:
    :show-inheritance:
