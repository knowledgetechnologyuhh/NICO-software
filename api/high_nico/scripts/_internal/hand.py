# Copyright (C) 2016 Marcus Soll
#
# This file is part of highNICO.
#
# highNICO is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# highNICO is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with highNICO. If not, see <http://www.gnu.org/licenses/>.

import logging
import time

def openHand(robot, handName, speed=10, percentage=1.0):
    """
    Opens the specified hand. handName can be 'RHand' or 'LHand'
    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param speed: Speed at which hand should open. Default: 10
    :type speed: int
    :param percentage: Percentage hand should open. 0.0 < percentage <= 1.0
    :type percentage: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if not (0.0 < percentage <= 1.0):
        logging.critical('percentage (%f) out of bounds' % percentage)
        return

    if handName == 'RHand':
        robot.r_indexfingers_x.compliant = False
        robot.r_indexfingers_x.goal_speed = speed
        robot.r_indexfingers_x.goal_position = 130.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = speed
        robot.r_thumb_x.goal_position = 130.0 * percentage
        time.sleep(1)
        robot.r_indexfingers_x.compliant = True
        robot.r_thumb_x.compliant = True
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = speed
        robot.l_indexfingers_x.goal_position = 130.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = speed
        robot.l_thumb_x.goal_position = 130.0 * percentage
        time.sleep(1)
        robot.l_indexfingers_x.compliant = True
        robot.l_thumb_x.compliant = True
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return


def closeHand(robot, handName, speed=10, percentage=1.0):
    """
    Closes the specified hand. handName can be 'RHand' or 'LHand'
    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param speed: Speed at which hand should close. Default: 10
    :type speed: int
    :param percentage: Percentage hand should open. 0.0 < percentage <= 1.0
    :type percentage: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if not (0.0 < percentage <= 1.0):
        logging.critical('percentage (%f) out of bounds' % percentage)
        return

    if handName == 'RHand':
        robot.r_indexfingers_x.compliant = False
        robot.r_indexfingers_x.goal_speed = speed
        robot.r_indexfingers_x.goal_position = -130.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = speed
        robot.r_thumb_x.goal_position = -130.0 * percentage
        time.sleep(1)
        robot.r_indexfingers_x.compliant = True
        robot.r_thumb_x.compliant = True
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = speed
        robot.l_indexfingers_x.goal_position = -130.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = speed
        robot.l_thumb_x.goal_position = -130.0 * percentage
        time.sleep(1)
        robot.l_indexfingers_x.compliant = True
        robot.l_thumb_x.compliant = True
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def moveWrist(robot, handName, x, z, speed=10):
    """
    Moves the wrist of one hand to the given position. handName can be 'RHand' or 'LHand'
    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param x: Target x position in degree
    :type x: float
    :param z: Target x position in degree
    :type z: float
    :param speed: Speed at which hand should close. Default: 10
    :type speed: int
    :return: none
    """
    if robot is None:
        logging.critical('No robot provided')
        return
    if handName == 'RHand':
        robot.r_wrist_x.compliant = False
        robot.r_wrist_x.goal_speed = speed
        robot.r_wrist_x.goal_position = x
        robot.r_wrist_z.compliant = False
        robot.r_wrist_z.goal_speed = speed
        robot.r_wrist_z.goal_position = z
        time.sleep(1)
        robot.r_wrist_x.compliant = True
        robot.r_wrist_z.compliant = True
        pass
    elif handName == 'LHand':
        robot.l_wrist_x.compliant = False
        robot.l_wrist_x.goal_speed = speed
        robot.l_wrist_x.goal_position = x
        robot.l_wrist_z.compliant = False
        robot.l_wrist_z.goal_speed = speed
        robot.l_wrist_z.goal_position = z
        time.sleep(1)
        robot.l_wrist_x.compliant = True
        robot.l_wrist_z.compliant = True
        pass
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return
