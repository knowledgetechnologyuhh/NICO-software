#!/usr/bin/env python

import rospy
import math
from nicomsg.msg import sff
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import JointPosition


class JointController:

    def __init__(self, prefix):
        self.pub = rospy.Publisher("/nico/motion/setAngle", sff, queue_size=10)
        self.joint_srv = rospy.Service(
            prefix + "/goal_joint_space_path",
            SetJointPosition,
            self.set_joint_position,
        )
        rospy.loginfo(f"Started joint controller with prefix: {prefix}")

    def set_joint_position(self, request):
        joint_position = request.joint_position
        message = sff()
        for i, joint_name in enumerate(joint_position.joint_name):
            message.param1 = joint_name
            message.param2 = math.degrees(joint_position.position[i])
            message.param3 = 0.03
            self.pub.publish(message)
        return True


def main():
    node = rospy.init_node("joint_controller_node", anonymous=True)
    jc = JointController(rospy.get_param("~prefix", ""))
    rospy.spin()


if __name__ == "__main__":
    main()
