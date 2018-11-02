#!/usr/bin/env python

from nicoface.FaceExpression import faceExpression
import nicomsg.msg as msg
import rospy

class NicoRosFaceExpression:
    """
    The NicoRosFaceExpression class exposes the functions of :class:`nicoface.FaceExpression` to ROS
    """

    def __init__(self, devicename='/dev/ttyACM0'):
        """
        NicoRosFaceExpression provides :class:`nicoface.FaceExpression` functions over ROS

        :param devicename: name of the device on the serial (default '/dev/ttyACM0')
        :type devicename: str
        """
        # init face
	#Erik: Adapted to automatic device detection
        #self.face = faceExpression(devicename)
	self.face = faceExpression()

        # init ROS
        rospy.init_node('faceexpression', anonymous=True)

        # setup subscriber
        rospy.Subscriber('nico/faceExpression/sendMouth', msg.affffa, self._ROSPY_sendMouth)
        rospy.Subscriber('nico/faceExpression/sendEyebrow', msg.sffff, self._ROSPY_sendEyebrow)
        rospy.Subscriber('nico/faceExpression/sendFaceExpression', msg.s, self._ROSPY_sendFaceExpression)

        rospy.spin()

    def _ROSPY_sendMouth(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.gen_mouth`

        :param message: ROS message
        :type message: nicomsg.msg.affffa
        """
        if len(message.param1)>1:
            self.face.mouth = self.face.gen_mouth((message.param1[0].param1, message.param1[0].param2, message.param1[0].param3, message.param1[0].param4),
                                                    (message.param1[1].param1, message.param1[1].param2, message.param1[1].param3, message.param1[1].param4))
        else:
            self.face.mouth = self.face.gen_mouth((message.param1[0].param1, message.param1[0].param2, message.param1[0].param3, message.param1[0].param4))
        self.face.send("m")

    def _ROSPY_sendEyebrow(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.gen_eyebrowse`

        :param message: ROS message
        :type message: nicomsg.msg.sffff
        """
        if message.param1 == "l":
            self.face.left = self.face.gen_eyebrowse((message.param2, message.param3, message.param4, message.param5), type=message.param1)
        else:
            self.face.right = self.face.gen_eyebrowse((message.param2, message.param3, message.param4, message.param5), type=message.param1)
        self.face.send(message.param1)

    def _ROSPY_sendFaceExpression(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.sendFaceExpression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.face.sendFaceExpression(message.param1)

if __name__ == '__main__':
    #import argparse

    #parser = argparse.ArgumentParser()
    #parser.add_argument('--device', nargs='?', default='/dev/ttyACM0',
    #help="serial device the sensor is connected with (e.g. /dev/ttyACM0)")
    #parser.add_argument('--device', nargs='?', default='auto_detection_of_device',
    #help="serial device the sensor is connected with (e.g. /dev/ttyACM0)")	

    #args = parser.parse_args()

    NicoRosFaceExpression()
