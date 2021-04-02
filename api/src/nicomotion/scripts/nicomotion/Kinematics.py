#!/usr/bin/env python

import logging
import math
import time
from os.path import abspath, dirname

import numpy as np

from nicomotion import Motion
from ._nicomotion_internal.gaikpy import chain, chain_definitions
from ._nicomotion_internal.gaikpy import robot as visualizer
from ikpy.utils import geometry


class Kinematics(object):
    """The Kinematics class can be used to control the arms of the NICO robot
    with inverse kinematics."""

    def __init__(
        self,
        robot,
        urdf=((dirname(abspath(__file__)) + "/urdf/kinematics.urdf")),
        visualizer=None,
    ):
        """
        The Kinematics class can be used to control the arms of the NICO robot
        with inverse kinematics.

        :param robot: Motion object that controls the robot
        :type robot: nicomotion.Motion
        :param visualizer: Visualizer obj[])ect
        :type visualizer: nicomotion.Visualizer
        """
        self.logger = logging.getLogger(__name__)
        self.rightchain = chain.Chain.from_urdf_file(
            urdf,
            base_elements=[
                "torso:11",
                "r_shoulder_z",
                "right_shoulder:11",
                "r_shoulder_y",
                "right_collarbone:11",
                "r_arm_x",
                "right_upper_arm:11",
                "r_elbow_y",
                "right_lower_arm:11",
                "r_wrist_z",
                "right_wrist:11",
                "r_wrist_x",
                "right_palm:11",
                "r_ringfingers_x",
            ],
            active_joints_mask=[
                False,
                True,
                True,
                True,
                True,
                True,
                True,
                False,
                False,
                False,
            ],
            name="right_arm",
        )
        self.leftchain = chain.Chain.from_urdf_file(
            urdf,
            base_elements=[
                "torso:11",
                "l_shoulder_z",
                "left_shoulder:11",
                "l_shoulder_y",
                "left_collarbone:11",
                "l_arm_x",
                "left_upper_arm:11",
                "l_elbow_y",
                "left_lower_arm:11",
                "l_wrist_z",
                "left_wrist:11",
                "l_wrist_x",
                "left_palm:11",
                "l_ringfingers_x",
            ],
            active_joints_mask=[
                False,
                True,
                True,
                True,
                True,
                True,
                True,
                False,
                False,
                False,
            ],
            name="left_arm",
        )

        self.robot = robot
        self.visualizer = visualizer

    def calculate_target_angles(
        self,
        arm_name,
        pos_x,
        pos_y,
        pos_z,
        roll,
        pitch,
        yaw,
        origin_matrix=None,
        distance_acc=0.01,
        orientation_acc=0.1,
        orientation_weight=0.3,
        num_generations=100,
    ):
        """
        Computes the motor angles for the given goal position and orientation
        of the end effector using a genetic algorithm

        :param arm_name: Name of the arm to move ("left" or "right")
        :type arm_name: str
        :param pos_x: target x position in meters
        :type pos_x: float
        :param pos_y: target y position in meters
        :type pos_y: float
        :param pos_z: target z position in meters
        :type pos_z: float
        :param roll: target rotation around x-axis in degrees
        :type roll: float
        :param pitch: target rotation around y-axis in degrees
        :type pitch: float
        :param yaw: target rotation around z-axis in degrees
        :type yaw: float
        :param origin_file: .csv file containing position and rotation values
                            to use as costum origin
        :type origin_file: str
        :param distance_acc: Minimal distance to goal position that the
                             solution should achieve (in meters)
        :type distance_acc: float
        :param orientation_acc: Minimal distance to goal orientation that the
                                solution should achieve (in radians?)
        :type orientation_acc: float
        :param orientation_weight: Contribution of orientation distance on the
                                   fitness score
                                   (distance_weight = 1 - orientation_weight)
        :type orientation_weight: float
        :param num_generations: Maximum number of generations until the genetic
                                algorithm is stopped if no solution is found
        :type num_generations: int

        :return: target angles of the motor joints (in degrees)
        :rtype: float
        """
        prefix = arm_name[0].lower()
        if prefix == "l":
            active_chain = self.leftchain
        elif prefix == "r":
            active_chain = self.rightchain
        else:
            self.logger.error("Unknown arm name '%s'", arm_name)
            exit(1)

        if origin_matrix is not None:
            origin_pos, origin_rot = geometry.from_transformation_matrix(origin_matrix)
            origin_pos = origin_pos[:-1]
        else:
            origin_pos = np.array([0, 0, 0])
            origin_rot = np.eye(3)

        start_angles = [
            0,
            self.robot.getAngle(prefix + "_shoulder_z"),
            self.robot.getAngle(prefix + "_shoulder_y"),
            self.robot.getAngle(prefix + "_arm_x"),
            self.robot.getAngle(prefix + "_elbow_y"),
            self.robot.getAngle(prefix + "_wrist_z"),
            self.robot.getAngle(prefix + "_wrist_x"),
            0,
            0,
            0,
        ]
        start_angles = map(math.radians, start_angles)

        # orientation
        roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])
        rpyM = geometry.rpy_matrix(roll, pitch, yaw)
        rpyM = np.dot(rpyM, origin_rot)
        # position
        pos_x, pos_y, pos_z = origin_pos + np.array([pos_x, pos_y, pos_z])
        # calculate target angles
        frame_target = geometry.to_transformation_matrix([pos_x, pos_y, pos_z], rpyM)
        self.logger.info("Calculating target angles")
        target_angles = active_chain.inverse_kinematics(
            frame_target,
            initial_position=start_angles,
            method="ga_simple",
            include_orientation=True,
            dist_acc=distance_acc,
            or_acc=orientation_acc,
            orientation_weight=orientation_weight,
            numGenerations=num_generations,
        )
        self.logger.debug("Calculation done")
        if self.visualizer is not None:
            self.visualizer.set_target_frame(frame_target)
            names = active_chain.get_all_active_joint_names()
            angles = active_chain.active_from_full(target_angles)
            self.visualizer.set_angles_radians(names, angles)
        return map(math.degrees, target_angles)

    def visualize_pose(
        self,
        arm_name,
        pos_x,
        pos_y,
        pos_z,
        roll,
        pitch,
        yaw,
        origin_file=None,
        distance_acc=0.01,
        orientation_acc=0.1,
        orientation_weight=0.3,
        num_generations=100,
    ):
        """
        Computes and visualizes the inverse kinematics without executing the
        movement on the robot

        :param arm_name: Name of the arm to move ("left" or "right")
        :type arm_name: str
        :param pos_x: target x position in meters
        :type pos_x: float
        :param pos_y: target y position in meters
        :type pos_y: float
        :param pos_z: target z position in meters
        :type pos_z: float
        :param roll: target rotation around x-axis in degrees
        :type roll: float
        :param pitch: target rotation around y-axis in degrees
        :type pitch: float
        :param yaw: target rotation around z-axis in degrees
        :type yaw: float
        :param origin_file: .csv file containing position and rotation values
                            to use as costum origin
        :type origin_file: str
        :param distance_acc: Minimal distance to goal position that the
                             solution should achieve (in meters)
        :type distance_acc: float
        :param orientation_acc: Minimal distance to goal orientation that the
                                solution should achieve (in radians?)
        :type orientation_acc: float
        :param orientation_weight: Contribution of orientation distance on the
                                   fitness score
                                   (distance_weight = 1 - orientation_weight)
        :type orientation_weight: float
        :param num_generations: Maximum number of generations until the genetic
                                algorithm is stopped if no solution is found
        :type num_generations: int
        """
        if origin_file:
            origin_matrix = self.load_end_effector_origin(origin_file)
        else:
            origin_matrix = np.eye(4)
        self.calculate_target_angles(
            arm_name,
            pos_x,
            pos_y,
            pos_z,
            roll,
            pitch,
            yaw,
            origin_matrix,
            distance_acc,
            orientation_acc,
            orientation_weight,
            num_generations,
        )

    def move_relative_to_origin_matrix(
        self,
        arm_name,
        pos_x,
        pos_y,
        pos_z,
        roll,
        pitch,
        yaw,
        origin_matrix=None,
        distance_acc=0.01,
        orientation_acc=0.1,
        orientation_weight=0.3,
        num_generations=100,
    ):
        target_angles = self.calculate_target_angles(
            arm_name,
            pos_x,
            pos_y,
            pos_z,
            roll,
            pitch,
            yaw,
            origin_matrix,
            distance_acc,
            orientation_acc,
            orientation_weight,
            num_generations,
        )
        prefix = arm_name[0].lower()
        self.logger.info(
            "Setting motor angles of '{}': {}".format(
                arm_name,
                zip(
                    [
                        prefix + "_shoulder_z",
                        prefix + "_shoulder_y",
                        prefix + "_arm_x",
                        prefix + "_elbow_y",
                        prefix + "_wrist_z",
                        prefix + "_wrist_x",
                    ],
                    target_angles[1:],
                ),
            )
        )
        # move motors
        vel = 0.02
        self.robot.setAngle(prefix + "_shoulder_z", target_angles[1], vel)
        self.robot.setAngle(prefix + "_shoulder_y", target_angles[2], vel)
        self.robot.setAngle(prefix + "_arm_x", target_angles[3], vel)
        self.robot.setAngle(prefix + "_elbow_y", target_angles[4], vel)
        if self.robot._vrep:
            self.robot.setAngle(prefix + "_wrist_z", -target_angles[5], vel)
        else:
            self.robot.setAngle(prefix + "_wrist_z", target_angles[5] * 2, vel)
        self.robot.setAngle(prefix + "_wrist_x", target_angles[6], vel)

    def move_to(
        self,
        arm_name,
        pos_x,
        pos_y,
        pos_z,
        roll,
        pitch,
        yaw,
        origin_file=None,
        distance_acc=0.01,
        orientation_acc=0.1,
        orientation_weight=0.2,
        num_generations=100,
    ):
        """
        Computes and executes inverse kinematics for the given arm to the given
        absolute position and orientation vectors. An origin file can be used
        to define a custom (0,0,0) point, otherwise the center of the torso
        is used.

        :param arm_name: Name of the arm to move ("left" or "right")
        :type arm_name: str
        :param pos_x: target x position in meters
        :type pos_x: float
        :param pos_y: target y position in meters
        :type pos_y: float
        :param pos_z: target z position in meters
        :type pos_z: float
        :param roll: target rotation around x-axis in degrees
        :type roll: float
        :param pitch: target rotation around y-axis in degrees
        :type pitch: float
        :param yaw: target rotation around z-axis in degrees
        :type yaw: float
        :param origin_file: .csv file containing position and rotation values
                            to use as costum origin
        :type origin_file: str
        :param distance_acc: Minimal distance to goal position that the
                             solution should achieve (in meters)
        :type distance_acc: float
        :param orientation_acc: Minimal distance to goal orientation that the
                                solution should achieve (in radians?)
        :type orientation_acc: float
        :param orientation_weight: Contribution of orientation distance on the
                                   fitness score
                                   (distance_weight = 1 - orientation_weight)
        :type orientation_weight: float
        :param num_generations: Maximum number of generations until the genetic
                                algorithm is stopped if no solution is found
        :type num_generations: int
        """
        if origin_file:
            origin_matrix = self.load_end_effector_origin(origin_file)
        else:
            origin_matrix = np.eye(4)
        self.move_relative_to_origin_matrix(
            arm_name,
            pos_x,
            pos_y,
            pos_z,
            roll,
            pitch,
            yaw,
            origin_matrix,
            distance_acc,
            orientation_acc,
            orientation_weight,
            num_generations,
        )

    def move_relative(
        self,
        arm_name,
        pos_x,
        pos_y,
        pos_z,
        roll,
        pitch,
        yaw,
        distance_acc=0.01,
        orientation_acc=0.1,
        orientation_weight=0.3,
        num_generations=100,
    ):
        """
        Computes and executes inverse kinematics for the given arm to the given
        position and orientation vectors relative to its current end effector.
        (wrist_x of the respective hand)

        :param arm_name: Name of the arm to move ("left" or "right")
        :type arm_name: str
        :param pos_x: target x position in meters
        :type pos_x: float
        :param pos_y: target y position in meters
        :type pos_y: float
        :param pos_z: target z position in meters
        :type pos_z: float
        :param roll: target rotation around x-axis in degrees
        :type roll: float
        :param pitch: target rotation around y-axis in degrees
        :type pitch: float
        :param yaw: target rotation around z-axis in degrees
        :type yaw: float
        :param distance_acc: Minimal distance to goal position that the
                             solution should achieve (in meters)
        :type distance_acc: float
        :param orientation_acc: Minimal distance to goal orientation that the
                                solution should achieve (in radians?)
        :type orientation_acc: float
        :param orientation_weight: Contribution of orientation distance on the
                                   fitness score
                                   (distance_weight = 1 - orientation_weight)
        :type orientation_weight: float
        :param num_generations: Maximum number of generations until the genetic
                                algorithm is stopped if no solution is found
        :type num_generations: int
        """
        self.logger.info(
            "Moving position of '%s' by %s and changing orientation by %s",
            arm_name,
            ", ".join(map(str, [pos_x, pos_y, pos_z])),
            ", ".join(map(str, [roll, pitch, yaw])),
        )
        origin_matrix = self.get_current_end_effector_transformation(arm_name)
        self.move_relative_to_origin_matrix(
            arm_name,
            pos_x,
            pos_y,
            pos_z,
            roll,
            pitch,
            yaw,
            origin_matrix,
            distance_acc,
            orientation_acc,
            orientation_weight,
            num_generations,
        )

    def get_current_end_effector_transformation(self, arm_name):
        """
        Computes end effector position and orientation from the forward
        kinematics of the arms current pose

        :param arm_name: Name of the arm to move ("left" or "right")
        :type arm_name: str

        :return: 4x4 transformation matrix of the endeffector
        :rtype: np.array
        """
        prefix = arm_name[0].lower()
        if prefix == "l":
            active_chain = self.leftchain
        elif prefix == "r":
            active_chain = self.rightchain
        else:
            self.logger.error("Unknown arm name '%s'", arm_name)
            exit(1)
        start_angles = [
            0,
            self.robot.getAngle(prefix + "_shoulder_z"),
            self.robot.getAngle(prefix + "_shoulder_y"),
            self.robot.getAngle(prefix + "_arm_x"),
            self.robot.getAngle(prefix + "_elbow_y"),
            self.robot.getAngle(prefix + "_wrist_z"),
            self.robot.getAngle(prefix + "_wrist_x"),
            0,
            0,
            0,
        ]
        if not self.robot.getVrep:
            start_angles[5] = start_angles[5] / 2.0
        start_angles = map(math.radians, start_angles)
        fwk = active_chain.forward_kinematics(start_angles, full_kinematics=False)
        self.logger.info(
            "Current end effector of '{}':\n{}".format(active_chain.name, fwk)
        )
        return fwk

    def save_end_effector_transformation(self, arm_name, path="position.npy"):
        """
        Saves current end effector transformation matrix of the given arm
        as .npy file to use it as origin.

        :param arm_name: Name of the arm to move ("left" or "right")
        :type arm_name: str
        :param path: Path of the saved file
        :type path: str
        """
        trans_matrix = self.get_current_end_effector_transformation(arm_name)
        self.logger.info("Saving end effector of '{}'...".format(arm_name))
        np.save(path, trans_matrix)
        self.logger.info(
            "Saved end effector transformation matrix of "
            + "'{}' as '{}'".format(arm_name, path)
        )

    def load_end_effector_origin(self, path):
        """
        Loads a previously saved end effector transformation matrix from a
        .npy file

        :param path: Path of the .npy file
        :type path: str

        :return: dict with "pos_x", "pos_y", "pos_z" in meters and
                 "roll", "pitch", "yaw" in radians
        :rtype: dict(str: float)
        """
        self.logger.info("Loading origin...")
        trans_matrix = np.load(path)
        self.logger.debug("{} loaded from {}".format(trans_matrix, path))
        self.logger.info("Successfully loaded '{}'".format(path))
        return trans_matrix
