from os.path import abspath, dirname
from time import sleep

from nicomotion.Motion import Motion

robot = Motion(
    dirname(abspath(__file__))
    + "/../../../../json/nico_humanoid_upper_rh7d.json",
    vrep=False)

robot.setAngle("l_shoulder_y", -22.5, 0.05)
robot.setAngle("l_elbow_y", 67.5, 0.05)
robot.setAngle("l_arm_x", 33.75, .05)
sleep(.5)
robot.setAngle("l_shoulder_z", -67.5, .05)
# robot.setAngle("l_wrist_z", -45, 1.)  # funktioniert nicht
robot._robot.l_wrist_z.compliant = False  # funktioniert...
robot._robot.l_wrist_z.goal_position = -45
robot.thumbsUp("LHand", .5)
sleep(4)
robot.setAngle("l_shoulder_z", 0, .05)
# robot.setAngle("l_wrist_z", 0, 1.)  # funktioniert nicht
robot._robot.l_wrist_z.goal_position = 0  # funktioniert...
robot.openHand("LHand", .5)
sleep(1.25)
robot.setAngle("l_shoulder_y", 0, 0.05)
robot.setAngle("l_elbow_y", 0, 0.05)
robot.setAngle("l_arm_x", 0, .05)
sleep(2)

robot.disableTorqueAll()
