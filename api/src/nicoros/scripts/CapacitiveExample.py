#!/usr/bin/env python

import nicomsg.msg as msg
import rospy


def callback(data):
    rospy.loginfo("%f, %f, %f, %f", data.param1, data.param2, data.param3, data.param4)


def listener():
    """
    Simple subscriber for capacitive sensor topic
    """
    rospy.init_node("capacitive_example", anonymous=True)

    rospy.Subscriber("nico/CapacitiveSensors/sensor_readings", msg.ffff, callback)

    rospy.spin()


if __name__ == "__main__":
    listener()
