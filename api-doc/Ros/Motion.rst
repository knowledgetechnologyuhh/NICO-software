rosnico - Motion
****************

The Motion class allows control over the NICO motors through ROS.

To start the interface run:

.. code-block:: bash
   
   rosrun nicoros Motion.py

Many options are available to change the behaviour of RosNico. For a list of them run:

.. code-block:: bash
   
   rosrun nicoros Motion.py -h

Exposed topics
##############
The following ROS topics are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/motion):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/changeAngle                            | nicomsg/sff                                    | Changes the angle of a single joint. Parameters:                                      |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. Angle                                                                             |
|                                                |                                                |  3. Fraction of max speed                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/closeHand                              | nicomsg/s                                      | Closes the hand. Parameters:                                                          |
|                                                |                                                |  1. "RHand" or "LHand"                                                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/disableForceControl                    | nicomsg/empty                                  | Disables force control for all joints                                                 |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/disableForceControlSingleJoint         | nicomsg/s                                      | Disables force control for a single joint. Parameters:                                |
|                                                |                                                |  1. Name of joint                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/enableForceControl                     | nicomsg/i                                      | Enables force control on all supported joints. Parameters:                            |
|                                                |                                                |  1. Goal force                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/enableForceControlSingleJoint          | nicomsg/si                                     | Enables force control on a single joint. Parameters:                                  |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. Goal force                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/openHand                               | nicomsg/s                                      | Opems the hand. Parameters:                                                           |
|                                                |                                                |  1. "RHand" or "LHand"                                                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setAngle                               | nicomsg/sff                                    | Sets the angle of a single joint. Parameters:                                         |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. angle                                                                             |
|                                                |                                                |  3. Fraction of max speed                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setMaximumSpeed                        | nicomsg/f                                      | Sets the maximum allowed speed (in fraction of maximum possible speed). Parameters:   |
|                                                |                                                |  1. Maximum speed                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setPID                                 | nicomsg/sfff                                   | Sets the PID control of a single joint. Parameters:                                   |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. *p* (proportional band)                                                           |
|                                                |                                                |  3. *i* (integral action)                                                             |
|                                                |                                                |  4. *d* (derivative action)                                                           |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/setStifftness                          | nicomsg/sf                                     | Sets the stifftness of a single joint. Parameters:                                    |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. stifftness                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Exposed services
################

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico):

+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| ROS service name                               | Service type                                   | Short description                                                          |
+================================================+================================================+============================================================================+
| $PREFIX/getAngle                               | nicomsg/GetValue                               | Returns the angle of a single joint                                        | 
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getAngleLowerLimit                     | nicomsg/GetValue                               | Returns the lower limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getAngleUpperLimit                     | nicomsg/GetValue                               | Returns the upper limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getCurrent                             | nicomsg/GetValue                               | Returns the present currency of a single joint                             |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getJointNames                          | nicomsg/GetNames                               | Returns a list with the names of all joints                                |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getPID                                 | nicomsg/GetPID                                 | Returns the PID control of a single joint                                  |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getSensorNames                         | nicomsg/GetNames                               | Returns a list with the names of all sensors                               |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getStifftness                          | nicomsg/GetValue                               | Returns the stifftness of a single joint                                   |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getTemperature                         | nicomsg/GetValue                               | Returns the present temperature of a single joint                          |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/getTorqueLimit                         | nicomsg/GetValue                               | Returns the torque limit of a single joint                                 |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+

Class documentation
###################

.. automodule:: Motion
    :members:
    :undoc-members:
    :show-inheritance:
