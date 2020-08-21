#!/usr/bin/env python

import nicomsg.msg as msg
import numpy as np
import rospy
from threading import Semaphore

try:
    import queue
except ImportError:
    import Queue as queue

from nicoface.FaceExpression import faceExpression


class NicoRosFaceExpression:
    """
    The NicoRosFaceExpression class exposes the functions of
    :class:`nicoface.FaceExpression` to ROS
    """

    def __init__(self, devicename=None, simulation=False):
        """
        NicoRosFaceExpression provides :class:`nicoface.FaceExpression`
        functions over ROS

        :param devicename: name of the device on the serial
                           (e.g. '/dev/ttyACM0', autodetected if None)
        :type devicename: str
        :param simulation: whether to use simulated or real face
        :type simulation: bool
        """
        # init face
        self.face = faceExpression(devicename, simulation)

        # thread managament
        self.mutex = Semaphore()
        self.pause = Semaphore(0)
        self.queue = queue.Queue(maxsize=1)

        # init ROS
        rospy.init_node("faceexpression", anonymous=True)

        # setup subscriber
        rospy.Subscriber(
            "nico/faceExpression/sendMouth", msg.affffa, self._ROSPY_sendMouth
        )
        rospy.Subscriber(
            "nico/faceExpression/sendEyebrow", msg.sffff, self._ROSPY_sendEyebrow
        )
        rospy.Subscriber(
            "nico/faceExpression/send_wavelet_face",
            msg.wavelet_face,
            self._ROSPY_wavelet_custom,
        )
        rospy.Subscriber(
            "nico/faceExpression/morph_wavelet_face",
            msg.wavelet_face,
            self._ROSPY_wavelet_morph,
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

        # execute callbacks in main thread
        while not rospy.is_shutdown():
            if not self.queue.empty():
                func, args = self.queue.get_nowait()
                func(*args)
                self.pause.release()

    def _ROSPY_sendMouth(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.gen_mouth`

        :param message: ROS message
        :type message: nicomsg.msg.affffa
        """
        self.mutex.acquire()
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
        self.queue.put((self.face.send, ("m",)))
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_sendEyebrow(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.gen_eyebrowse`

        :param message: ROS message
        :type message: nicomsg.msg.sffff
        """
        self.mutex.acquire()
        self.face.gen_eyebrowse(
            (message.param2, message.param3, message.param4, message.param5),
            type=message.param1,
        )
        self.queue.put((self.face.send, (message.param1,)))
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_sendFaceExpression(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.sendFaceExpression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.mutex.acquire()
        self.queue.put((self.face.sendFaceExpression, (message.param1,)))
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_trained_expression(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.sendTrainedFaceExpression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.mutex.acquire()
        self.queue.put((self.face.sendTrainedFaceExpression, (message.param1,)))
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_morph(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.sendFaceExpression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.mutex.acquire()
        # TODO other parameters?
        self.queue.put((self.face.morph_face_expression, (message.param1,)))
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_polynomial_preset(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.send_morphable_face_expression`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.mutex.acquire()
        self.queue.put((self.face.send_morphable_face_expression, (message.param1,)))
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_polynomial_mouth(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.generate_polynomial_mouth`

        :param message: ROS message
        :type message: nicomsg.msg.polynomial_mouth
        """
        self.mutex.acquire()
        self.face.generate_polynomial_mouth(
            message.degs1,
            message.degs2,
            message.x_shift,
            message.crop_left,
            message.crop_right,
        )
        self.queue.put((self.face.send, ()))
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_polynomial_eyebrow(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.generate_polynomial_eyebrow`

        :param message: ROS messarospy.Publisher(
                "nico/faceExpression/capacitive_readings", msg.ffff, queue_size=1
            )ge
        :type message: nicomsg.msg.polynomial_eyebrow
        """
        self.mutex.acquire()
        self.face.generate_polynomial_eyebrow(
            message.degs,
            message.x_shift,
            message.crop_left,
            message.crop_right,
            message.left,
        )
        self.queue.put((self.face.send, ()))
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_polynomial_custom(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.send_polynomial_face`

        :param message: ROS message
        :type message: nicomsg.msg.polynomial_face
        """
        mouth = message.mouth
        left = message.brow_left
        right = message.brow_right
        self.mutex.acquire()
        self.queue.put(
            (
                self.face.send_polynomial_face,
                (
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
                ),
            )
        )
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_polynomial_morph(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.morph_polynomial_face`

        :param message: ROS message
        :type message: nicomsg.msg.polynomial_face
        """
        mouth = message.mouth
        left = message.brow_left
        right = message.brow_right
        self.mutex.acquire()
        self.queue.put(
            (
                self.face.morph_polynomial_face,
                (
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
                ),
            )
        )
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_wavelet_custom(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.send_wavelet_face`

        :param message: ROS message
        :type message: nicomsg.msg.wavelet_face
        """
        mouth = message.mouth
        left = message.brow_left
        right = message.brow_right
        self.mutex.acquire()
        self.queue.put(
            (
                self.face.send_wavelet_face,
                (
                    (
                        mouth.param1[0].param1,
                        mouth.param1[0].param2,
                        mouth.param1[0].param3,
                        mouth.param1[0].param4,
                    ),
                    (
                        mouth.param1[1].param1,
                        mouth.param1[1].param2,
                        mouth.param1[1].param3,
                        mouth.param1[1].param4,
                    ),
                    (left.param1, left.param2, left.param3, left.param4),
                    (right.param1, right.param2, right.param3, right.param4),
                ),
            )
        )
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_wavelet_morph(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.morph_wavelet_face`

        :param message: ROS message
        :type message: nicomsg.msg.wavelet_face
        """
        mouth = message.mouth
        left = message.brow_left
        right = message.brow_right
        self.mutex.acquire()
        self.queue.put(
            (
                self.face.morph_wavelet_face,
                (
                    (
                        mouth.param1[0].param1,
                        mouth.param1[0].param2,
                        mouth.param1[0].param3,
                        mouth.param1[0].param4,
                    ),
                    (
                        mouth.param1[1].param1,
                        mouth.param1[1].param2,
                        mouth.param1[1].param3,
                        mouth.param1[1].param4,
                    ),
                    (left.param1, left.param2, left.param3, left.param4),
                    (right.param1, right.param2, right.param3, right.param4),
                ),
            )
        )
        self.pause.acquire()
        self.mutex.release()

    def _ROSPY_bitmap(self, message):
        """
        Callback handle for :meth:`nicoface.FaceExpression.send_bitmap_face`

        :param message: ROS message
        :type message: nicomsg.msg.bitmap_face
        """
        mouth = np.frombuffer(message.mouth, dtype="uint8").reshape(
            message.mouth_shape_0, message.mouth_shape_1
        )
        left = np.frombuffer(message.brow_left, dtype="uint8").reshape(
            message.brow_shape_0, message.brow_shape_1
        )
        right = np.frombuffer(message.brow_right, dtype="uint8").reshape(
            message.brow_shape_0, message.brow_shape_1
        )
        self.mutex.acquire()
        self.queue.put((self.face.send_bitmap_face, (left, right, mouth)))
        self.pause.acquire()
        self.mutex.release()


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
    parser.add_argument(
        "-s",
        action="store_true",
        help="Enables simulated mode, where faces are shown as image instead",
    )

    args = parser.parse_args()

    NicoRosFaceExpression(args.device, args.s)
