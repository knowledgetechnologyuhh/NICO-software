#!/usr/bin/env python

"""
A simple test of the follow-joint-trajectory action without using MoveIt!.
The script creates a simple joint trajectory and sends it in form of an action to the TrajectoryServer.
"""

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import std_msgs.msg
import rospy
import math

#Test joint l_elbow_y

print 'init'
rospy.init_node('trajectory_test')
client = actionlib.SimpleActionClient('/nico/trajectory', control_msgs.msg.FollowJointTrajectoryAction)
print 'waiting'
client.wait_for_server()

print 'building trajectory'
start_time = rospy.Time.now() + rospy.Duration(5)

trajectory = control_msgs.msg.FollowJointTrajectoryGoal()
joint_tolerance = control_msgs.msg.JointTolerance()
joint_tolerance.name = 'l_elbow_y'
joint_tolerance.position = 10.0
joint_goal_tolerance = control_msgs.msg.JointTolerance()
joint_goal_tolerance.name = 'l_elbow_y'
joint_goal_tolerance.position = 1.0
trajectory.path_tolerance += [ joint_tolerance ]
trajectory.goal_tolerance += [ joint_goal_tolerance ]
trajectory.goal_time_tolerance = rospy.Duration(10)

trajectory_points = trajectory_msgs.msg.JointTrajectory()
trajectory_points.joint_names = [ 'l_elbow_y' ]
trajectory_points.header = std_msgs.msg.Header()
trajectory_points.header.stamp = start_time
trajectory_points.points = []

p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [ math.radians(0.0) ]
p.time_from_start = rospy.Duration(10)
trajectory_points.points += [ p ]

p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [ math.radians(50.0) ]
p.time_from_start = rospy.Duration(20)
trajectory_points.points += [ p ]

p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [ math.radians(-50.0) ]
p.time_from_start = rospy.Duration(30)
trajectory_points.points += [ p ]

p = trajectory_msgs.msg.JointTrajectoryPoint()
p.positions = [ math.radians(100.0) ]
p.time_from_start = rospy.Duration(40)
trajectory_points.points += [ p ]

trajectory.trajectory = trajectory_points

print 'sending request'
client.send_goal(trajectory)
print 'waiting for results'
client.wait_for_result()
print client.get_result()
