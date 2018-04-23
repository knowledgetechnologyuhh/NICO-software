# Erik Strahl

# GNU GPL License

from nicomotion import Motion

#Put the left arm in defined position
robot = Motion.Motion("../../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)

#set the robot to be compliant
#robot.disableTorqueAll()

robot.openHand('LHand', 0.5)
