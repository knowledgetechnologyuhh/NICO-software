#!/usr/bin/env python

import nicomsg.msg as msg
import rospy
from nicotouch import OptoforceMultichannel


class NicoRosOptoforceMultichannel:
    """
    The NicoRosOptoforceMultichannel class publishes the raw and newton sensor
    data of multiple optoforce sensors connected to the same device via ROS
    """

    def __init__(self, ser_number):
        """
        The NicoRosOptoforceMultichannel class publishes the raw and newton
        sensor data of multiple optoforce sensors connected to the same device
        via ROS.

        :param port: serial device the sensor is connected with (e.g.
                     /dev/ttyACM0)
        :type port: str
        :param ser_number: serial number of the sensor
        :type ser_number: str
        """
        rospy.init_node("optoforce", anonymous=True)

        sensor = OptoforceMultichannel.OptoforceMultichannel(ser_number)

        raw_publishers = []
        newton_publishers = []
        if sensor._ser_number in OptoforceMultichannel.keys:
            for key in sensor._keys:
                if key is not None:
                    raw_publishers.append(
                        rospy.Publisher(
                            "nico/optoforce/%s/raw/%s" % (sensor._ser_number, key),
                            msg.fff,
                            queue_size=10,
                        )
                    )
                    newton_publishers.append(
                        rospy.Publisher(
                            "nico/optoforce/%s/newton/%s" % (sensor._ser_number, key),
                            msg.fff,
                            queue_size=10,
                        )
                    )
        else:
            rospy.logwarn(
                "Can't initialize newton publishers - Unknown serial number %s",
                sensor._ser_number,
            )
            for key in sensor._keys:
                raw_publishers.append(
                    rospy.Publisher(
                        "nico/optoforce/%s/raw/%s" % (sensor._ser_number, key),
                        msg.fff,
                        queue_size=10,
                    )
                )

        while not rospy.is_shutdown():
            raw_values = sensor.get_sensor_values_raw()
            for i in range(len(raw_publishers)):
                message = msg.fff()
                message.param1, message.param2, message.param3 = raw_values["forces"][
                    sensor._keys[i]
                ]
                raw_publishers[i].publish(message)
            if newton_publishers:
                newton = sensor.get_sensor_values()
                for i in range(len(newton_publishers)):
                    message = msg.fff()
                    message.param1, message.param2, message.param3 = newton["forces"][
                        sensor._keys[i]
                    ]
                    newton_publishers[i].publish(message)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--serial", nargs="?", default=None, help="serial number of the sensor"
    )

    args = parser.parse_args()

    try:
        NicoRosOptoforceMultichannel(args.serial)
    except Exception as e:
        raise
