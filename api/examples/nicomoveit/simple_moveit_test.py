#!/usr/bin/env python

"""
This script is used to test MoveIt! without use of the MoveIt!-Wrapper
"""

import moveit_commander
import sys
import rospy
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("leftArm")

# print(robot.get_group_names())
# print(robot.get_current_state())

# This is the default path planner in Kinetic anyway, but not in Indigo
group.set_planner_id("RRTConnectkConfigDefault")

print("increase tolerance")
group.set_goal_position_tolerance(0.1)
group.set_goal_orientation_tolerance(0.2)

values = [
    -0.7417967536653856,
    2.2122617473316843,
    0.3553677613749635,
    -1.4112707151742188,
    -1.3299268114123495,
    -0.3222468796948495,
]
group.set_joint_value_target(values)
print("plan")
plan = group.plan()
print("execute")
group.execute(plan)
rospy.sleep(2)


print("Known constraints: ")
print(group.get_known_constraints())

for i in range(10):
    pose_target = group.get_random_pose()
    # print(pose_target)
    group.set_pose_target(pose_target)
    group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.x = 0.158434282046
pose_target.orientation.y = 0.807295567431
pose_target.orientation.z = 0.258856217432
pose_target.orientation.w = 0.506128347138
pose_target.position.x = 0.390251923048
pose_target.position.y = 0.339816569177
pose_target.position.z = 0.760846647956

group.set_pose_target(pose_target)
print("1")
group.go()
rospy.sleep(2)

pose_target.orientation.x = -0.973379789458
pose_target.orientation.y = -0.0139847757448
pose_target.orientation.z = 0.00340166727638
pose_target.orientation.w = 0.228745798172
pose_target.position.x = 0.0718405144708
pose_target.position.y = 0.153894236006
pose_target.position.z = 0.395156039253

group.set_pose_target(pose_target)
print("2")
group.go()
