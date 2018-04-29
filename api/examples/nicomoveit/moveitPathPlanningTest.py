#!/usr/bin/env python

"""
Make sure that a ROS instance is running before this script is executed!

Simple example how to perform collision free motion planning using MoveIt! through the moveitWrapper that was created for NICO.
"""

import time
import math
from nicomoveit import moveitWrapper

# Creating a groupHandle object that we can use to access the MoveIt! capabilities for one specific planning group
groupName = "leftArm"
leftArm = moveitWrapper.groupHandle(groupName,vrep=True,robotMotorFile='nico_humanoid_legged_with_hands_mod-vrep_with_legs.json',vrepScene='NICO-seated-with-table.ttt',visualize=True)

# if you already executed 'roslaunch nicoros nicoros_moveit_visual.launch' or 'roslaunch nicoros nicoros_moveit.launch' in a seperate terminal most of the parameters are obsolete:
#leftArm = moveitWrapper.groupHandle(groupName, visualize=True)

# To enable collision free motion planning, it is necessary to model the obstacles in the workspace correctly
# These meshes have been extracted from V-Rep. Unfortunately it might be necessary to adapt the spawn position to put the obstacle at its appropriate place.
time.sleep(3)
leftArm.addMesh("table","table.stl",position=[0,0.0,0.175])
time.sleep(3)
leftArm.addMesh("chair","chair.stl",position=[0.015,0.0,0.175])
time.sleep(3)

p_x = 0.43085057313
p_y = 0.134845567708
p_z = 0.802798506593 
# simple motion planning to a cartesian position with certain orientation of the last link of the planning group
# possible orientations are: sideGrasp, topGrasp, or any quaternion
leftArm.moveToPose([p_x,p_y,p_z],"sideGrasp")

#leftArm.group.set_max_velocity_scaling_factor(0.001)

target = [-0.1969, -1.4026, -0.1552, 0.3835, 0.0, 0.0]
for idx in range(0, len(target)):
  target[idx] = math.degrees(target[idx])
# motion planning to move to a position specified by a list of joint values (one for each joint of the planning group) in degrees
leftArm.moveToJointCoordinates(target)

# motion planning to a pose that is 10cm further to the right
leftArm.shiftPose(1, 0.1)

# move to the pose the arm had before we started motion planning
leftArm.toSafePosition()

