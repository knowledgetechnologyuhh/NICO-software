#!/usr/bin/env python

import nicomsg.msg as msg
import rospy
from nicoface.CapacitiveSensors import CapacitiveSensors


class NicoRosCapacitiveSensors:
    """
    The NicoRosCapacitiveSensors class exposes the functions of
    :class:`nicoface.CapacitiveSensors` to ROS
    """

    def __init__(self, devicename=None, rate=10):
        """
        NicoRosCapacitiveSensors provides :class:`nicoface.CapacitiveSensors`
        functions over ROS

        :param devicename: name of the device on the serial
                           (e.g. '/dev/ttyACM0', autodetected if None)
        :type devicename: str
        :param rate: publish rate of the capacitive readings
        :type rate: int
        """
        # init sensor
        self.cap_sensor = CapacitiveSensors(devicename)

        # init ROS
        rospy.init_node("capacitive_sensors", anonymous=True)

        # setup publisher
        pub = rospy.Publisher(
            "nico/CapacitiveSensors/sensor_readings", msg.ffff, queue_size=1
        )
        publish_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            sensor_readings = self.cap_sensor.getCapacitiveReadings()
            pub.publish(*sensor_readings)
            publish_rate.sleep()


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
        "--rate", nargs="?", help=("publish rate of the sensor readings"), default=10
    )

    args = parser.parse_args()

    NicoRosCapacitiveSensors(args.device, args.rate)
