#!/usr/bin/env python

from nicotouch.optoforcesensors import optoforce
import nicomsg.msg as msg
import rospy

class NicoRosOptoforce:
    """
    The NicoRosOptoforce class publishes the raw and newton sensor data of an optoforce sensor via ROS
    """
    def __init__(self, port, ser_number):
        """
        The NicoRosOptoforce class publishes the raw and newton sensor data of an optoforce sensor via ROS

        :param port: serial device the sensor is connected with (e.g. /dev/ttyACM0)
        :type port: str
        :param ser_number: serial number of the sensor
        :type ser_number: str
        """
        sensor = optoforce(port,ser_number)

        publisher_raw = rospy.Publisher('nico/optoforce/'+ser_number+'/raw', msg.iii, queue_size=10)
        publisher_newton = rospy.Publisher('nico/optoforce/'+ser_number+'/newton', msg.fff, queue_size=10)

        rospy.init_node('optoforce', anonymous=True)

        msg_raw = msg.iii()
        msg_newton = msg.fff()

        while not rospy.is_shutdown():
            msg_raw.param1, msg_raw.param2, msg_raw.param3 = sensor.get_sensor_values_raw()
            msg_newton.param1, msg_newton.param2, msg_newton.param3 = sensor.get_sensor_values()

            publisher_raw.publish(msg_raw)
            publisher_newton.publish(msg_newton)


if __name__ == '__main__':
    import sys
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--serial', nargs='?', default='DSE0A125',
    help="serial number of the sensor")
    parser.add_argument('--device', nargs='?', default='/dev/ttyACM0',
    help="serial device the sensor is connected with (e.g. /dev/ttyACM0)")

    args = parser.parse_args()

    try:
        NicoRosOptoforce(args.device,args.serial)
    except Exception as e:
        raise
