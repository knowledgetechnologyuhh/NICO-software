from nicomotion import Motion

fMS = 0.01
fMS_hand=1.0
robot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=False)

#set the robot to be compliant
robot.disableTorqueAll()


#robot.enableForceControl("r_wrist_z", 50)
#robot.enableForceControl("r_wrist_x", 50)

#robot.enableForceControl("r_indexfingers_x", 50)
#robot.enableForceControl("r_thumb_x", 50)

robot.setAngle("r_wrist_z", 0, fMS_hand)
robot.setAngle("r_wrist_x", 0, fMS_hand)


#open hand    
robot.openHand('RHand', fractionMaxSpeed=fMS_hand)
#move to start position (example: ([-19.12,  11.65, -30.37,  79.52, -38.37,  -8.66]))

raw_input()


#open hand and move to start position
robot.closeHand('RHand', fractionMaxSpeed=fMS_hand)

raw_input()
robot.setAngle("r_wrist_z", 13.32, fMS_hand)
robot.setAngle("r_wrist_x", -67.47, fMS_hand)

raw_input()
robot.setAngle("r_wrist_z", 0, fMS_hand)
robot.setAngle("r_wrist_x", 0, fMS_hand)

#set the robot to be compliant
robot.disableTorqueAll()












