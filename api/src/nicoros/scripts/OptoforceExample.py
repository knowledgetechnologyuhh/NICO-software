#!/usr/bin/env python

import nicomsg.msg as msg
import rospy


def callback(data):
    rospy.loginfo("x: %i, y: %i, z: %i", data.param1, data.param2, data.param3)


def listener():
    """
    Simple subscriber for optoforce sensor topic
    """
    rospy.init_node('optoforce_example', anonymous=True)

    rospy.Subscriber("nico/optoforce/DSE0A125/raw", msg.iii, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
