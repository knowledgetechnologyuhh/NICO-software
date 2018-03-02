nicoros - FaceExpression
************************

The FaceExpression class allows to manipulate the facial expression of NICO through ROS.

To start the interface run:

.. code-block:: bash

   rosrun nicoros FaceExpression.py --devicename='/dev/ttyACM0'

Exposed topics
==============

The following ROS services are exposed by the class - where $PREFIX stands for the prefix of the topics/services (Default: /nico/faceExpression/):

+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| ROS topic name                                 | Message type                                   | Short description                                                                     |
+================================================+================================================+=======================================================================================+
| $PREFIX/sendMouth                              | nicomsg/affffa                                 | Sends up to two wavelets to create the mouth. Parameters:                             |
|                                                |                                                |   1. Array of (ystr,yoff,xstr,xoff) tuples                                            |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/sendEyebrow                            | nicomsg/sffff                                  | Sends a wavelet to create one of the eybrows. Parameters:                             |
|                                                |                                                |   1. 'l' or 'r'                                                                       |
|                                                |                                                |   2. stretch in y position                                                            |
|                                                |                                                |   3. offset in y position                                                             |
|                                                |                                                |   4. stretch in x position                                                            |
|                                                |                                                |   5. offset in x position                                                             |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+
| $PREFIX/sendFaceExpression                     | nicomsg/s                                      | Sets facial expression to one of the presets. Parameters:                             |
|                                                |                                                |   1. 'happiness','sadness','anger','disgust','surprise','fear','neutral', or 'clean'  |
+------------------------------------------------+------------------------------------------------+---------------------------------------------------------------------------------------+

Class documentation
===================

.. automodule:: FaceExpression
    :members:
    :undoc-members:
    :show-inheritance:
