#!/usr/bin/env python

import cv_bridge
import cv2
import nicomsg.msg
import nicomsg.srv
import rospy
import sensor_msgs.msg
from std_srvs.srv import Empty
import time


class PyrepExample(object):
    """
    Example class for usage of nicoros Pyrep module. To run the Pyrep module
    use "rosrun nicoros Pyrep.py" (-h for list of optional parameters)
    """

    def __init__(self):
        rospy.init_node("PyrepExample", anonymous=True)
        # service proxis to control simulation
        self.start_sim = rospy.ServiceProxy("/nico/pyrep/startSimulation", Empty)
        self.stop_sim = rospy.ServiceProxy("/nico/pyrep/stopSimulation", Empty)
        self.step_sim = rospy.ServiceProxy("/nico/pyrep/nextSimulationStep", Empty)
        # motion
        self.angle_publisher = rospy.Publisher(
            "/nico/pyrep/motion/setAngle", nicomsg.msg.sff, queue_size=10
        )
        # vision
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber(
            "/nico/pyrep/vision/left",
            sensor_msgs.msg.Image,
            self.callback,
            queue_size=1,
        )

    def callback(self, image_message):
        """
        Callback function of subscribed topic. Converts image into greyscale
        """

        cv_image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


if __name__ == "__main__":
    try:
        example = PyrepExample()
        example.start_sim()

        time.sleep(1)

        example.angle_publisher.publish("head_z", -90, 0.05)
        example.angle_publisher.publish("r_arm_x", -90, 0.05)

        for i in range(50):
            example.step_sim()

        time.sleep(1)

        time.sleep(1)

        example.stop_sim()
    except rospy.ROSInterruptException:
        pass
