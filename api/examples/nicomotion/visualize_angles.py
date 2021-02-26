from os.path import dirname, abspath
from nicomotion import Visualizer
import logging

robot = Visualizer.Visualizer()

# set one angle
robot.set_angle("l_shoulder_z", -10)

# set multiple angles at once
names, angles = zip(
    ["l_shoulder_y", 20],
    ["l_arm_x", 20],
    ["l_elbow_y", 100],
    ["l_wrist_z", 0],
    ["l_wrist_x", 0],
)
robot.set_angles(names, angles)

# visualize target position
robot.set_target_position(x=0.3, y=0.2, z=0.6)

# with rotation
robot.set_target_position(x=0.3, y=0.2, z=0.6, roll=22.5, pitch=0, yaw=0)
