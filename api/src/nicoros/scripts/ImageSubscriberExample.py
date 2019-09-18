#!/usr/bin/env python

import cv2
import cv_bridge
import nicomsg.srv
import rospy
import sensor_msgs.msg


class ImageSubscriber(object):
    """
    Example class for usage of nicoros Vision module. To run the Vision module
    use "rosrun nicoros Vision.py" (-h for list of optional parameters)
    """

    def __init__(self):
        rospy.init_node('ImageSubscriber', anonymous=True)

        try:
            # example service call from within code. this should only be used
            # to set pan during runtime. Initial pan can be set on launch by
            # running Vision.py with the --pan <value> parameter or by giving a
            # settings json with the --settings_file option
            panvalue = 0
            setPan = rospy.ServiceProxy(
                'nico/vision/setPan', nicomsg.srv.SetIntValue)
            response = setPan(panvalue)
            if response.success:
                print("I just set the pan value to {}".format(panvalue))
        except rospy.ServiceException as e:
            "Service call failed: %s" % e

        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber("/nico/vision/left",
                                           sensor_msgs.msg.Image,
                                           self.callback, queue_size=1)

    def callback(self, image_message):
        '''
        Callback function of subscribed topic. Converts image into greyscale
        '''

        cv_image = self.bridge.imgmsg_to_cv2(image_message, "bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


if __name__ == '__main__':
    subscriber = ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
