#!/usr/bin/env python
import numpy as np
import rospy
import nicomsg.msg as msg

delay = 1.0

rospy.init_node("face_test_node")

# default presets (won't work in simulated mode)

rospy.loginfo("Sending 'happiness' preset")

pub = rospy.Publisher(
    "nico/faceExpression/sendFaceExpression", msg.s, latch=True, queue_size=0
)

pub.publish("happiness")

rospy.sleep(delay)

# polynomial based faces

rospy.loginfo("Sending morphable 'surprise' polynomial preset")

pub = rospy.Publisher(
    "nico/faceExpression/send_morphable_expression", msg.s, latch=True, queue_size=0
)

pub.publish("surprise")

rospy.sleep(delay)

rospy.loginfo("Morphing face to 'sadness' preset")

pub = rospy.Publisher(
    "nico/faceExpression/morph_face_expression", msg.s, latch=True, queue_size=0
)

pub.publish("sadness")

rospy.sleep(delay)

rospy.loginfo("Sending custom polynomial face")

# send mouth and brows individually using seperate publishers:

# pub_mouth = rospy.Publisher(
#     "nico/faceExpression/send_polynomial_mouth",
#     msg.polynomial_mouth,
#     latch=True,
#     queue_size=0,
# )
# pub_eye = rospy.Publisher(
#     "nico/faceExpression/send_polynomial_eyebrow", msg.polynomial_eyebrow, queue_size=0
# )
#
# mouth = msg.polynomial_mouth([7, 0, -0.25, 0, 0], [0, 0, 0.1, 0, 0], 7.45, 3, 3)
# brow_left = msg.polynomial_eyebrow([2, 0, 0.2, 0, 0], 3.45, 0, 0, True)
# brow_right = msg.polynomial_eyebrow([2, 0, 0.2, 0, 0], 3.45, 0, 0, False)
#
# pub_mouth.publish(mouth)
# pub_eye.publish(brow_left)
# pub_eye.publish(brow_right)
#
# rospy.sleep(delay)

pub = rospy.Publisher(
    "nico/faceExpression/send_polynomial_face",
    msg.polynomial_face,
    latch=True,
    queue_size=0,
)

message = msg.polynomial_face()

message.mouth = msg.polynomial_mouth([7, 0, -0.25, 0, 0], [0, 0, 0.1, 0, 0], 7.45, 3, 3)
message.brow_left = msg.polynomial_eyebrow([2, 0, 0.2, 0, 0], 3.45, 0, 0, True)
message.brow_right = msg.polynomial_eyebrow([2, 0, 0.2, 0, 0], 3.45, 0, 0, False)

pub.publish(message)

rospy.sleep(delay)

rospy.loginfo("Morphing face to custom polynomial face")

pub = rospy.Publisher(
    "nico/faceExpression/morph_polynomial_face",
    msg.polynomial_face,
    latch=True,
    queue_size=0,
)

message = msg.polynomial_face()

message.mouth = msg.polynomial_mouth(
    [4, 1.75, 0, -0.05, 0], [4, 1.75, 0, -0.05, 0], 7.45, 0, 0
)
message.brow_left = msg.polynomial_eyebrow([1, 0, 0.075, 0, 0], 0, 0, 0, True)
message.brow_right = msg.polynomial_eyebrow([4, -0.75, 0.05, 0, 0], 0, 0, 0, False)

pub.publish(message)

rospy.sleep(delay)

# wavelet based faces

rospy.loginfo("Sending trained 'happiness' wavelet preset")

pub = rospy.Publisher(
    "nico/faceExpression/send_trained_expression", msg.s, latch=True, queue_size=0
)

pub.publish("happiness")

rospy.sleep(delay)

rospy.loginfo("Sending custom wavelet face")

# send mouth and brows individually using:
#
# pub_mouth = rospy.Publisher(
#     "nico/faceExpression/sendMouth", msg.affffa, latch=True, queue_size=0
# )
# pub_eye = rospy.Publisher("nico/faceExpression/sendEyebrow", msg.sffff, queue_size=0)
#
# mouth = msg.affffa((msg.ffff(1.0, -0.6, 0.9, 0.02), msg.ffff(1.0, -0.6, 0.9, 0.02),))
# brow_left = msg.sffff("l", -0.5, 0.2, 1.0, 0.0)
# brow_right = msg.sffff("r", -0.5, 0.2, 1.0, 0.0)
#
# pub_mouth.publish(mouth)
# pub_eye.publish(brow_left)
# pub_eye.publish(brow_right)
#
# rospy.sleep(delay)

pub = rospy.Publisher(
    "nico/faceExpression/send_wavelet_face", msg.wavelet_face, latch=True, queue_size=0
)

message = msg.wavelet_face()

message.mouth = msg.affffa(
    (msg.ffff(1.0, -0.6, 0.9, 0.02), msg.ffff(1.0, -0.6, 0.9, 0.02),)
)
message.brow_left = msg.ffff(-0.5, 0.2, 1.0, 0.0)
message.brow_right = msg.ffff(-0.5, 0.2, 1.0, 0.0)

pub.publish(message)

rospy.sleep(delay)

rospy.loginfo("Morphing face into custom wavelet face")

pub = rospy.Publisher(
    "nico/faceExpression/morph_wavelet_face", msg.wavelet_face, latch=True, queue_size=0
)

message = msg.wavelet_face()

message.mouth = msg.affffa(
    (msg.ffff(0.6, 0.1, 1.0, 0.025), msg.ffff(-1.1, 0.2, 0.95, 0.025),)
)
message.brow_left = msg.ffff(1.2, -0.5, 1.0, 0.0)
message.brow_right = msg.ffff(1.2, -0.5, 1.0, 0.0)

pub.publish(message)

rospy.sleep(delay)

# Directly setting bitmaps

rospy.loginfo("Sending bitmap face")

pub = rospy.Publisher(
    "nico/faceExpression/send_bitmap", msg.bitmap_face, latch=True, queue_size=0
)

message = msg.bitmap_face()

message.brow_left = tuple(
    np.array(
        [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 1, 0, 0, 0],
            [0, 1, 1, 0, 0, 1, 1, 0],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
        ],
        dtype="uint8",
    ).flatten()
)

message.brow_right = tuple(
    np.array(
        [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 1, 0, 0, 0],
            [0, 1, 1, 0, 0, 1, 1, 0],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
        ],
        dtype="uint8",
    ).flatten()
)

message.mouth = np.array(
    [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
    ],
    dtype="uint8",
).tobytes()

pub.publish(message)

rospy.sleep(delay)

# rospy.spin()
