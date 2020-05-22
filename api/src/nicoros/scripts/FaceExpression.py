#!/usr/bin/env python

import nicomsg.msg as msg
import rospy
import threading
from nicoface.FaceExpression import faceExpression
from nicoface.CapacitiveSensors import CapacitiveSensors


class NicoRosFaceExpression:
    """
    The NicoRosFaceExpression class exposes the functions of
    :class:`nicoface.FaceExpression` to ROS
    """

    def __init__(self, devicename=None, simulation=False, sensor_rate=10):
        """
        NicoRosFaceExpression provides :class:`nicoface.FaceExpression`
        functions over ROS

        :param devicename: name of the device on the serial
                           (e.g. '/dev/ttyACM0', autodetected if None)
        :type devicename: str
        :param sensor_rate: publish rate of the capacitive readings
        :type devicename: int
        """
        # init face
        self.face = faceExpression(devicename, simulation)
        self.cap_sensor = CapacitiveSensors(devicename)

        # init ROS
        rospy.init_node("faceexpression", anonymous=True)

        # setup publisher for CapacitiveSensors
        if self.cap_sensor.getCapacitiveReadings() and not simulation:
            self.cap_thread = threading.Thread(
                target=self._capacitive_publisher, args=(sensor_rate,)
            )
            self.cap_thread.start()
        else:
            rospy.loginfo("No capacitive pads detected, skipping publisher")

        # setup subscriber
        rospy.Subscriber(
            "nico/faceExpression/sendMouth", msg.affffa, self._ROSPY_sendMouth
        )
        rospy.Subscriber(
            "nico/faceExpression/sendEyebrow", msg.sffff, self._ROSPY_sendEyebrow
        )
        rospy.Subscriber(
            "nico/faceExpression/sendFaceExpression",
            msg.s,
            self._ROSPY_sendFaceExpression,
        )
        rospy.Subscriber(
            "nico/faceExpression/send_trained_expression",
            msg.s,
            self._ROSPY_trained_expression,
        )
        rospy.Subscriber(
            "nico/faceExpression/morph_face_expression", msg.s, self._ROSPY_morph,
        )
        rospy.Subscriber(
            "nico/faceExpression/send_morphable_expression",
            msg.s,
            self._ROSPY_polynomial_preset,
        )
        rospy.Subscriber(
            "nico/faceExpression/send_polynomial_mouth",
            msg.polynomial_mouth,
            self._ROSPY_polynomial_mouth,
        )
        rospy.Subscriber(
            "nico/faceExpression/send_polynomial_eyebrow",
            msg.polynomial_eyebrow,
            self._ROSPY_polynomial_eyebrow,
        )
        rospy.Subscriber(
            "nico/faceExpression/send_polynomial_face",
            msg.polynomial_face,
            self._ROSPY_polynomial_custom,
        )
        rospy.Subscriber(
            "nico/faceExpression/morph_polynomial_face",
            msg.polynomial_face,
            self._ROSPY_polynomial_morph,
        )
        rospy.Subscriber(
            "nico/faceExpression/send_bitmap", msg.bitmap_face, self._ROSPY_bitmap,
        )

        rospy.spin()

    def _capacitive_publisher(self, rate):
        pub = rospy.Publisher(
            "nico/faceExpression/capacitive_readings", msg.ffff, queue_size=1
        )
        publish_rate = rospy.Rate(rate)  # 10hz
        while not rospy.is_shutdown():
            sensor_readings = self.cap_sensor.getCapacitiveReadings()
            pub.publish(sensor_readings)
            publish_rate.sleep()

    def _ROSPY_sendMouth(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.gen_mouth`

        :param message: ROS message
        :type message: nicomsg.msg.affffa
        """
        if len(message.param1) > 1:
            self.face.gen_mouth(
                (
                    message.param1[0].param1,
                    message.param1[0].param2,
                    message.param1[0].param3,
                    message.param1[0].param4,
                ),
                (
                    message.param1[1].param1,
                    message.param1[1].param2,
                    message.param1[1].param3,
                    message.param1[1].param4,
                ),
            )
        else:
            self.face.gen_mouth(
                (
                    message.param1[0].param1,
                    message.param1[0].param2,
                    message.param1[0].param3,
                    message.param1[0].param4,
                )
            )
        self.face.send("m")

    def _ROSPY_sendEyebrow(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.gen_eyebrowse`

        :param message: ROS message
        :type message: nicomsg.msg.sffff
        """
        if message.param1 == "l":
            self.face.gen_eyebrowse(
                (message.param2, message.param3, message.param4, message.param5),
                type=message.param1,
            )
        else:
            self.face.gen_eyebrowse(
                (message.param2, message.param3, message.param4, message.param5),
                type=message.param1,
            )
        self.face.send(message.param1)

    def _ROSPY_sendFaceExpression(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.sendFaceExpression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.face.sendFaceExpression(message.param1)

    def _ROSPY_trained_expression(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.sendTrainedFaceExpression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.face.sendTrainedFaceExpression(message.param1)

    def _ROSPY_morph(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.sendFaceExpression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        # TODO other parameters?
        self.face.morph_face_expression(message.param1)

    def _ROSPY_polynomial_preset(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.send_morphable_face_expression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.face.send_morphable_face_expression(message.param1)

    def _ROSPY_polynomial_mouth(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.generate_polynomial_mouth`

        :param message: ROS message
        :type message: nicomsg.msg.polynomial_mouth
        """
        self.face.generate_polynomial_mouth(**message)  # TODO Test if that works

    def _ROSPY_polynomial_eyebrow(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.generate_polynomial_eyebrow`

        :param message: ROS messarospy.Publisher(
                "nico/faceExpression/capacitive_readings", msg.ffff, queue_size=1
            )ge
        :type message: nicomsg.msg.polynomial_eyebrow
        """
        self.face.generate_polynomial_eyebrow(**message)  # TODO Test if that works

    def _ROSPY_polynomial_custom(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.send_polynomial_face`

        :param message: ROS message
        :type message: nicomsg.msg.polynomial_face
        """
        mouth = message.mouth
        left = message.brow_left
        right = message.brow_right
        self.face.send_polynomial_face(
            mouth.degs1,
            mouth.degs2,
            mouth.x_shift,
            mouth.crop_left,
            mouth.crop_right,
            left.degs,
            left.x_shift,
            left.crop_left,
            left.crop_right,
            right.degs,
            right.x_shift,
            right.crop_left,
            right.crop_right,
        )

    def _ROSPY_polynomial_morph(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.morph_polynomial_face`

        :param message: ROS message
        :type message: nicomsg.msg.polynomial_face
        """
        mouth = message.mouth
        left = message.brow_left
        right = message.brow_right
        self.face.morph_polynomial_face(
            mouth.degs1,
            mouth.degs2,
            mouth.x_shift,
            mouth.crop_left,
            mouth.crop_right,
            left.degs,
            left.x_shift,
            left.crop_left,
            left.crop_right,
            right.degs,
            right.x_shift,
            right.crop_left,
            right.crop_right,
        )

    def _ROSPY_bitmap(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.send_bitmap_face`

        :param message: ROS message
        :type message: nicomsg.msg.bitmap_face
        """
        # FIXME might have to reshape arrays
        self.face.send_bitmap_face(message.brow_left, message.brow_right, message.mouth)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--device",
        nargs="?",
        help=(
            "serial device the sensor is connected with "
            + "(e.g. /dev/ttyACM0), autodetected if not set"
        ),
    )

    args = parser.parse_args()

    NicoRosFaceExpression(args.device)
