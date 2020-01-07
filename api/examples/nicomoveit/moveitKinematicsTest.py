#!/usr/bin/env python

"""
Make sure that a ROS instance is running before this script is executed!

Examples how to compute forward and inverse kinematics for NICO using the moveitWrapper.
"""

import time
from nicomoveit import moveitWrapper

time.sleep(1)

groupName = "leftArm"
leftArm = moveitWrapper.groupHandle(
    groupName,
    kinematicsOnly=True,
    vrep=False,
    robotMotorFile="nico_humanoid_upper.json",
    # robotMotorFile="nico_humanoid_legged_with_hands_mod-vrep.json",
)

p_x = 0.43085057313
p_y = 0.134845567708
p_z = 0.802798506593

o_x = 0.587817663801
o_y = 0.489151593275
o_z = 0.466306524902
o_w = 0.444701402915

# standard IK computation using a cartesian position and a quaternion specifying the orientation of the last link of the planning group
ik_solution = leftArm.computeIK([p_x, p_y, p_z], [o_x, o_y, o_z, o_w])
if ik_solution is not None:
    position, orientation = leftArm.computeFK(ik_solution)
    ik_solution = leftArm.computeIK(position, orientation)
if ik_solution is not None:
    # sideGrasp and topGrasp can be used alternatively to an actual quaternion to specify the orientation
    # also using initialJointValues the starting joint values can be specified that might influence the solution depedend on the used IK solver
    ik_solution = leftArm.computeIK(
        [p_x, p_y, p_z], "sideGrasp", initialJointValues=ik_solution
    )
if ik_solution is not None:
    # instead of using the last link to reach the specified pose, any other link of the planning group can be used when specified with the 'tip' argument
    position, orientation = leftArm.computeFK(ik_solution, tip="left_lower_arm:11")
# with ignoreCollisions it is irrelevant wether the solution is actually accessable or not
ik_solution = leftArm.computeIK(
    [p_x, p_y, p_z], "sideGrasp", ignoreCollisions=True, tip="left_palm:11"
)
