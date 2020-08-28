#!/usr/bin/env python

import nicomsg.msg as msg
import rospy


def create_callback(channel):
    def callback(data):
        rospy.loginfo(
            "%s: x: %f, y: %f, z: %f", channel, data.param1, data.param2, data.param3
        )

    return callback


def listener(ser_number, channel):
    """
    Simple subscriber for optoforce sensor topic
    """
    rospy.init_node("optoforce_example", anonymous=True)

    rospy.Subscriber(
        "nico/optoforce/{}/raw/{}".format(ser_number, channel),
        msg.fff,
        create_callback(channel),
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "serial",
        help="serial number of the sensor (hint: use "
        "'rostopic list' and look for optoforce topics)",
    )
    parser.add_argument(
        "--keys",
        nargs="+",
        help="channel identifiers of each sensor (hint: use "
        "'rostopic list' and look for optoforce topics)",
    )
    args = parser.parse_args()

    if not args.keys:
        from nicotouch.OptoforceMultichannel import keys as known_sensors

        if args.serial in known_sensors:
            keys = known_sensors[args.serial]
        else:
            rospy.logwarn(
                "Unknown serial '%s' and no keys specified - using default "
                + "keys '0', '1', '2', '3'",
                args.serial,
            )
            keys = ["0", "1", "2", "3"]
    else:
        keys = args.keys

    for channel in keys:
        if channel:
            listener(args.serial, channel)
    rospy.spin()
