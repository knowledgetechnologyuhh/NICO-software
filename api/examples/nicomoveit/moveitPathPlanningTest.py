#!/usr/bin/env python

import time
import math
from nicomoveit import moveitWrapper

groupName = "leftArm"
leftArm = moveitWrapper.groupHandle(groupName,vrep=True,robotMotorFile='nico_humanoid_legged_with_hands_mod-vrep_with_legs.json',vrepScene='NICO-seated-with-table.ttt',visualize=True)

# if you already executed 'roslaunch nicoros nicoros_moveit_visual.launch' or 'roslaunch nicoros nicoros_moveit.launch' in a seperate terminal most of the parameters are obsolete:
#leftArm = moveitWrapper.groupHandle(groupName, visualize=True)

time.sleep(3)
leftArm.addMesh("table","table.stl",position=[0,0.0,0.175])
time.sleep(3)
leftArm.addMesh("chair","chair.stl",position=[0.015,0.0,0.175])
time.sleep(3)

p_x = 0.43085057313
p_y = 0.134845567708
p_z = 0.802798506593 
leftArm.moveToPose([p_x,p_y,p_z],"sideGrasp")

#leftArm.group.set_max_velocity_scaling_factor(0.001)

target = [-0.1969, -1.4026, -0.1552, 0.3835, 0.0, 0.0]
for idx in range(0, len(target)):
  target[idx] = math.degrees(target[idx])

leftArm.moveToJointCoordinates(target)

leftArm.shiftPose(1, 0.1)

target2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # position with collision
leftArm.moveToJointCoordinates(target2)

