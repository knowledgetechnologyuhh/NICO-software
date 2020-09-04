#!/usr/bin/env python

# USAGE: Use script with yes or no as parameter
# python yesno.py yes
# python yesno.py no

# license removed for brevity
import time

import rospy
from nicomsg.msg import sff


def talker():
    pub = rospy.Publisher('/nico/motion/setAngle', sff, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    position = -20
    for i in range(6):
        position = position * -1
        pub.publish("head_y", position, 0.05)
        time.sleep(1)

    pub.publish("head_z", 0, 0.05)
    pub.publish("head_y", 0, 0.05)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
