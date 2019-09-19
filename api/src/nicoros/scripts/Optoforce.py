#!/usr/bin/env python

import nicomsg.msg as msg
import rospy
from nicotouch.optoforcesensors import optoforce


class NicoRosOptoforce:
    """
    The NicoRosOptoforce class publishes the raw and newton sensor data of an
    optoforce sensor via ROS
    """

    def __init__(self, ser_number):
        """
        The NicoRosOptoforce class publishes the raw and newton sensor data of
        an optoforce sensor via ROS

        :param port: serial device the sensor is connected with (e.g.
                     /dev/ttyACM0)
        :type port: str
        :param ser_number: serial number of the sensor
        :type ser_number: str
        """
        sensor = optoforce(ser_number)

        publisher_raw = rospy.Publisher(
            'nico/optoforce/' + sensor._ser_number + '/raw', msg.iii, queue_size=10)
        publisher_newton = rospy.Publisher(
            'nico/optoforce/' + sensor._ser_number + '/newton', msg.fff, queue_size=10)

        rospy.init_node('optoforce', anonymous=True)

        msg_raw = msg.iii()
        msg_newton = msg.fff()

        while not rospy.is_shutdown():
            msg_raw.param1, msg_raw.param2, msg_raw.param3 = \
                sensor.get_sensor_values_raw()
            msg_newton.param1, msg_newton.param2, msg_newton.param3 = \
                sensor.get_sensor_values()

            publisher_raw.publish(msg_raw)
            publisher_newton.publish(msg_newton)


if __name__ == '__main__':
    import sys
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--serial', nargs='?', default=None,
                        help="serial number of the sensor")

    args = parser.parse_args()

    try:
        NicoRosOptoforce(args.serial)
    except Exception as e:
        raise
