#!/usr/bin/env python

import time
import math
from nicomoveit import moveitWrapper

time.sleep(1)

groupName = "leftArm"
leftArm = moveitWrapper.groupHandle(groupName, kinematicsOnly=True, robotMotorFile='nico_humanoid_legged_with_hands_mod-vrep.json')

p_x = 0.43085057313
p_y = 0.134845567708
p_z = 0.802798506593 

o_x = 0.587817663801
o_y = 0.489151593275
o_z = 0.466306524902
o_w = 0.444701402915

ik_solution = leftArm.computeIK([p_x,p_y,p_z],[o_x,o_y,o_z,o_w])
if ik_solution is not None:
  position, orientation = leftArm.computeFK(ik_solution)
  ik_solution = leftArm.computeIK(position,orientation)
if ik_solution is not None:
  ik_solution = leftArm.computeIK([p_x,p_y,p_z], "sideGrasp", initialJointValues = ik_solution)
if ik_solution is not None:
  position, orientation = leftArm.computeFK(ik_solution, tip = "left_lower_arm:11")
ik_solution = leftArm.computeIK([p_x,p_y,p_z], "sideGrasp", ignoreCollisions = True, tip = "left_palm:11")
