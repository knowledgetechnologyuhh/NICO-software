#!/usr/bin/env python

import nicomsg.msg as msg
import rospy


def callback(data):
    rospy.loginfo("x: %i, y: %i, z: %i", data.param1, data.param2, data.param3)


def listener(ser_number):
    """
    Simple subscriber for optoforce sensor topic
    """
    rospy.init_node('optoforce_example', anonymous=True)

    rospy.Subscriber("nico/optoforce/{}/raw".format(ser_number), msg.iii, callback)

    rospy.spin()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('serial',
                        help="serial number of the sensor (hint: use "
                        "'rostopic list' and look for optoforce topics)")

    args = parser.parse_args()

    listener(args.serial)
