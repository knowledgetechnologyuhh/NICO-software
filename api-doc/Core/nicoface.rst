nicoface
**********

The **nicoface** package contains classes to control the facial expressions of the NICO robot.

faceExpression
##############

The faceExpression class allows manipulation of the facial expressions of the NICO robot via wavelets and preset expressions.

.. automodule:: nicoface.FaceExpression
    :members:
    :undoc-members:
    :show-inheritance:
    :private-members:

**Update 4/Dec/2019: contribution by Seed Robotics (www.seedorobotics.com)**

Extended the Arduino skecthes of the board contolling the head LED matrixes, to support reading of capacitive channels installed in the NICo head.

Extended the _faceExpression_ class with accompanying methods:

* getCapacitiveReadings() - returns a list with the capacitive readings. Use the /len()/ python operator to determine the size of the list. An empty object is returned if there is a communication failure or the feature is not suppported on the Arduino skecth running in the head.
* recallibrateCapacitivePads() - re-callibrates the zero/baseline reading on the capacitivePads. By default pads are callibrated on power up of the head board. Use this function only if you find significant deviations during operation.

For further information refer to the Arduino sketches.