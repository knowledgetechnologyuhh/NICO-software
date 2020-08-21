nicoros - Pyrep
****************

The Pyrep class allows control over motors and access to sensors of a NICO
simulated in CoppeliaSim through ROS. It combines functions of the Motion and
Vision modules.

Before running the node, make sure the required environment variables
LD_LIBRARY_PATH and QT_QPA_PLATFORM_PLUGIN_PATH are set properly.
To do so run:

.. code-block:: bash

   source api/pyrep_env.bash

To start the interface with default parameters run:

.. code-block:: bash

   rosrun nicoros Pyrep.py --vrep-scene=path/to/scene

For a full list of parameters run:

.. code-block:: bash

   rosrun nicoros Pyrep.py -h

Exposed topics
##############
The following ROS topics are exposed by the class - where $PREFIX stands for the
prefix of the topics/services (Default: /nico/pyrep), $MOTION is an extension of
the prefix for motion related topics (Default: /motion) and $VISION is the same
for vision related topics (Default: /vision):

Motion
------

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/$MOTION/changeAngle                    | nicomsg/sff                                    | Changes the angle of a single joint. Parameters:                                      |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. Angle                                                                             |
|                                                |                                                |  3. Fraction of max speed                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/closeHand                      | nicomsg/s                                      | Closes the hand. Parameters:                                                          |
|                                                |                                                |  1. "RHand" or "LHand"                                                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/disableTorque                  | nicomsg/s                                      | Disables torque on a single joint. Parameters:                                        |
|                                                |                                                |  1. Name of joint                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/disableTorqueAll               | nicomsg/empty                                  | Disables torque on all joints.                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/enableTorque                   | nicomsg/s                                      | Enables torque on a single joint. Parameters:                                         |
|                                                |                                                |  1. Name of joint                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/enableTorqueAll                | nicomsg/empty                                  | Enables torque on all joints.                                                         |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/openHand                       | nicomsg/s                                      | Opens the hand. Parameters:                                                           |
|                                                |                                                |  1. "RHand" or "LHand"                                                                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/setAngle                       | nicomsg/sff                                    | Sets the angle of a single joint. Parameters:                                         |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. angle                                                                             |
|                                                |                                                |  3. Fraction of max speed                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/setMaximumSpeed                | nicomsg/f                                      | Sets the maximum allowed speed (in fraction of maximum possible speed). Parameters:   |
|                                                |                                                |  1. Maximum speed                                                                     |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/setPID                         | nicomsg/sfff                                   | Sets the PID control of a single joint. Parameters:                                   |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. *p* (proportional band)                                                           |
|                                                |                                                |  3. *i* (integral action)                                                             |
|                                                |                                                |  4. *d* (derivative action)                                                           |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$MOTION/setStifftness                  | nicomsg/sf                                     | Sets the stifftness of a single joint. Parameters:                                    |
|                                                |                                                |  1. Name of joint                                                                     |
|                                                |                                                |  2. stifftness                                                                        |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Vision
------

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/$VISION/left                           | sensor_msgs/Image                              | Video stream of the left vision sensor (one frame per simulation step)                |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/$VISION/right                          | sensor_msgs/Image                              | Video stream of the right visio  sensor (one frame per simulation step)               |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+


Exposed services
################

The following ROS services are exposed by the class - where $PREFIX stands for the
prefix of the topics/services (Default: /nico/pyrep), $MOTION is an extension of
the prefix for motion related topics (Default: /motion) and $VISION is the same
for vision related topics (Default: /vision):


Pyrep
-----

+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| ROS service name                               | Service type                                   | Short description                                                          |
+================================================+================================================+============================================================================+
| $PREFIX/getPose                                | nicomsg/GetValues                              | Returns the the position of an object in the scene                         |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/nextSimulationStep                     | std_srvs/Empty                                 | Triggers the next simulation step.                                         |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/startSimulation                        | std_srvs/Empty                                 | Starts the simulation.                                                     |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/stopSimulation                         | std_srvs/Empty                                 | Stops the simulation.                                                      |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+


Motion
------

+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| ROS service name                               | Service type                                   | Short description                                                          |
+================================================+================================================+============================================================================+
| $PREFIX/$MOTION/getAngle                       | nicomsg/GetValue                               | Returns the angle of a single joint                                        |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getAngleLowerLimit             | nicomsg/GetValue                               | Returns the lower limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getAngleUpperLimit             | nicomsg/GetValue                               | Returns the upper limit of the angle for a single joint                    |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getConfig                      | nicomsg/GetString                              | Returns the JSON configuration of motors                                   |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getCurrent                     | nicomsg/GetValue                               | Returns the present currency of a single joint                             |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getJointNames                  | nicomsg/GetNames                               | Returns a list with the names of all joints                                |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getPID                         | nicomsg/GetPID                                 | Returns the PID control of a single joint                                  |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getStifftness                  | nicomsg/GetValue                               | Returns the stifftness of a single joint                                   |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getTemperature                 | nicomsg/GetValue                               | Returns the present temperature of a single joint                          |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getTorqueLimit                 | nicomsg/GetValue                               | Returns the torque limit of a single joint                                 |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+
| $PREFIX/$MOTION/getVrep                        | nicomsg/GetString                              | Returns if vrep simulation is used or not                                  |
+------------------------------------------------+------------------------------------------------+----------------------------------------------------------------------------+


Class documentation
###################

.. automodule:: Pyrep
    :members:
    :undoc-members:
    :show-inheritance:
