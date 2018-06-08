import random
import sys
import time
from os.path import abspath, dirname
from threading import Thread

from nicomotion import Motion


def current_reading(robot):
    while True:
        print("Thumb: {}, Finger: {}, Wrist: {}".format(
            robot.getCurrent("r_thumb_x"), robot.getCurrent(
                "r_indexfingers_x"), robot.getCurrent("r_wrist_x")
        ))
        time.sleep(.1)


if __name__ == '__main__':

    robot = Motion.Motion(dirname(abspath(__file__)) +
                          "/../../../json/nico_humanoid_upper.json",
                          vrep=False)
    hand = "RHand"

    t = Thread(group=None, target=current_reading, name=None,
               args=(robot,), kwargs=None, verbose=None)
    t.daemon = True
    t.start()

    poses = {
        "closeHand": robot.closeHand,
        "openHand": robot.openHand}

    if len(sys.argv) < 2:
        print("Please add at least one pose as argument. Multiple arguments " +
              "will be executed in sequence.\n Known poses are: \n{}".format(
                  ', '.join(poses.keys())))

    for pose in sys.argv[1:]:
        if pose not in poses.keys():
            print("Unknown pose {}, try one of the following: {}".format(
                pose, ', '.join(poses.keys())))
            break
        else:
            poses[pose](hand, .5)

        time.sleep(2.0)

    robot.setAngle("r_wrist_x", random.randint(-50, 35), 1.)
    time.sleep(2.)
