#!/usr/bin/env python

import time
import math
from nicomoveit import moveitWrapper
from nicomotion import Motion

time.sleep(1)
#robot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod.json",vrep=True,vrepScene='../../../v-rep/NICO-seated.ttt')
#robot = Motion.Motion("../../../json/nico_humanoid_legged_with_hands_mod-vrep.json",vrep=False)

# NICO-seated_easy_with_table.ttt
# PointingNico-new-version.ttt
# NICO-standing.ttt
# NICO-seated.ttt 

#joints = robot.getJointNames()
#print joints
#for joint in joints:
#  print robot.getAngle(joint)

moveitWrapper.sittingPosition()
time.sleep(3)
moveitWrapper.addMesh("table","table.stl",position=[0.0,0,0.175])
time.sleep(3)
moveitWrapper.addMesh("chair","chair.stl",position=[0.015,0,0.175])
time.sleep(3)

groupName = "leftArm"
# set realNICO to True, if real (non vrep) version is used
# set robot = None if movement should not be executed in VREP or on real NICO (so only in RViz)
robot = None
leftArm = moveitWrapper.groupHandle(groupName,robot, realNICO = False)


leftArm.setMaxSpeed(0.001)
leftArm.setMotionPlanner("RRTConnectkConfigDefault")

p_x = 0.43085057313
p_y = 0.134845567708
p_z = 0.802798506593 
leftArm.moveToPose([p_x,p_y,p_z],"sideGrasp")

o_x = 0.587817663801
o_y = 0.489151593275
o_z = 0.466306524902
o_w = 0.444701402915
leftArm.moveToPose([p_x,p_y,p_z],[o_x,o_y,o_z,o_w])

p_x = 0.18
p_y = 0.28 
p_z = 0.60
leftArm.pickAndPresent([p_x,p_y,p_z],"topGrasp",0.06)

p_x = 0.43085057313
p_y = 0.134845567708
p_z = 0.802798506593 
o_x = 0.587817663801
o_y = 0.489151593275
o_z = 0.466306524902
o_w = 0.444701402915
leftArm.moveToPose([p_x,p_y,p_z],[o_x,o_y,o_z,o_w])
leftArm.moveToPose([p_x,p_y,p_z],"sideGrasp")

#import benchmarkFramework
#bench = benchmarkFramework.groupHandle("leftArm")
#pose = bench.createRandomReachablePoses()
#leftArm.moveToJointCoordinates(pose[0])

leftArm.setOrientationTolerance(0.4)
leftArm.setPositionTolerance(0.01)
leftArm.moveToPose([p_x,p_y,p_z+0.07],[o_x,o_y,o_z,o_w])
time.sleep(4)
leftArm.computeCartesianPath(2,-0.06,False,[p_x,p_y,p_z+0.06],[o_x,o_y,o_z,o_w])
time.sleep(4)

leftArm.moveToDefault()
