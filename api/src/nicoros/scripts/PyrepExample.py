#!/usr/bin/env python

import time

import nicomsg.msg
import rospy
from std_srvs.srv import Empty


def talker():
    start_sim = rospy.ServiceProxy('/nico/motion/startSimulation', Empty)
    stop_sim = rospy.ServiceProxy('/nico/motion/stopSimulation', Empty)
    step_sim = rospy.ServiceProxy('/nico/motion/nextSimulationStep', Empty)
    angle_publisher = rospy.Publisher(
        '/nico/motion/setAngle', nicomsg.msg.sff, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    start_sim()

    time.sleep(1)

    angle_publisher.publish("head_z", -90, 0.05)
    angle_publisher.publish("r_shoulder_y", 90, 0.05)

    for i in range(50):
        step_sim()

    time.sleep(1)

    stop_sim()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
