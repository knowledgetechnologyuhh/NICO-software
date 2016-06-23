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

def openHand(robot, handName, fractionMaxSpeed=1.0, percentage=1.0):
    """
    Opens the specified hand. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
    :type fractionMaxSpeed: float
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
        robot.r_indexfingers_x.goal_speed = 10.0 * fractionMaxSpeed
        robot.r_indexfingers_x.goal_position = 130.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 10.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = 130.0 * percentage
        time.sleep(1)
        robot.r_indexfingers_x.compliant = True
        robot.r_thumb_x.compliant = True
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 10.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = 130.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 10.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = 130.0 * percentage
        time.sleep(1)
        robot.l_indexfingers_x.compliant = True
        robot.l_thumb_x.compliant = True
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return


def closeHand(robot, handName, fractionMaxSpeed=1.0, percentage=1.0):
    """
    Closes the specified hand. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should close. Default: 1.0
    :type fractionMaxSpeed: float
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
        robot.r_indexfingers_x.goal_speed = 10.0 * fractionMaxSpeed
        robot.r_indexfingers_x.goal_position = -130.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 10.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = -130.0 * percentage
        time.sleep(1)
        robot.r_indexfingers_x.compliant = True
        robot.r_thumb_x.compliant = True
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 10.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = -130.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 10.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = -130.0 * percentage
        time.sleep(1)
        robot.l_indexfingers_x.compliant = True
        robot.l_thumb_x.compliant = True
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return
