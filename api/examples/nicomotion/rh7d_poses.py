from nicomotion import Motion
import time
from os.path import dirname, abspath
import sys

if __name__ == '__main__':

    robot = Motion.Motion(dirname(abspath(__file__))+"/../../../json/rh7d_hands.json",vrep=False)
    hand = "RHand"

    poses = {
    "thumbsUp": robot.thumbsUp,
    "pointAt":robot.pointAt,
    "okSign":robot.okSign,
    "pinchToIndex":robot.pinchToIndex,
    "keyGrip":robot.keyGrip,
    "pencilGrip":robot.pencilGrip,
    "closeHand":robot.closeHand,
    "openHand":robot.openHand}


    if len(sys.argv) < 2:
        print("Please add at least one pose as argument. Multiple arguments will be executed in sequence.\n Known poses are: \n{}".format(', '.join(poses.keys())))

    for pose in sys.argv[1:]:
        if pose not in poses.keys():
            print("Unknown pose {}, try one of the following: {}".format(pose, ', '.join(poses.keys())))
            break
        else:
            poses[pose](hand)
        time.sleep(5.0)
