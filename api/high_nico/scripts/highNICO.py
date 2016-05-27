#!/usr/bin/env python

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
from time import sleep
import json

import pypot.robot
import pypot.vrep

class HighNico:
    """
    The HighNico class provides a high level interface to various functions of the NICO robot
    """

    def __init__(self, motor_config='config.json', vrep=False, vrep_host='127.0.0.1', vrep_port=19997, vrep_scene=None):
        """
        HighNico is a high level interface to the NICO robot,
        :param motor_config: motor config file (JSON format)
        :type motor_config: str
        :param vrep: If set to true VREP will be used instead of real robot
        :type vrep: bool
        :param vrep_host: Network address of VREP
        :type vrep_host: str
        :param vrep_port: Port of VREP
        :type vrep_port: int
        :param vrep_scene: Scene to load. Set to None to use current scene
        :type vrep_scene: str
        """
        self._highNICO_robot = None

        with open(motor_config, 'r') as config_file:
            config = json.load(config_file)

        if vrep:
            logging.info('highNICO: Using VREP')
            self._highNICO_robot = pypot.vrep.from_vrep(config, vrep_host, vrep_port, vrep_scene)
        else:
            logging.info('highNICO: Using robot')
            self._highNICO_robot = pypot.robot.from_config(config)

    def openHand(self, handName):
        """
        Opens the specified hand. handName can be 'RHand' or 'LHand'
        :param handName: Name of the hand (RHand, LHand)
        :return: None
        """
        if self._highNICO_robot is None:
            logging.critical('NICO not initialised')
            return

        if handName == 'RHand':
            self._highNICO_robot.r_indexfingers_x.compliant = False
            self._highNICO_robot.r_indexfingers_x.goal_speed = 10
            self._highNICO_robot.r_indexfingers_x.goal_position = -140
            self._highNICO_robot.r_thumb_x.compliant = False
            self._highNICO_robot.r_thumb_x.goal_speed = 10
            self._highNICO_robot.r_thumb_x.goal_position = -140
            sleep(1)
            self._highNICO_robot.r_indexfingers_x.compliant = True
            self._highNICO_robot.r_thumb_x.compliant = True
        elif handName == 'LHand':
            self._highNICO_robot.l_indexfingers_x.compliant = False
            self._highNICO_robot.l_indexfingers_x.goal_speed = 10
            self._highNICO_robot.l_indexfingers_x.goal_position = -140
            self._highNICO_robot.l_thumb_x.compliant = False
            self._highNICO_robot.l_thumb_x.goal_speed = 10
            self._highNICO_robot.l_thumb_x.goal_position = -140
            sleep(1)
            self._highNICO_robot.l_indexfingers_x.compliant = True
            self._highNICO_robot.l_thumb_x.compliant = True
        else:
            logging.warning('Unknown hand handle: %s' % handName)
            return

    def closeHand(self, handName):
        """
        Closes the specified hand. handName can be 'RHand' or 'LHand'
        :param handName: Name of the hand (RHand, LHand)
        :return: None
        """
        if self._highNICO_robot is None:
            logging.critical('NICO not initialised')
            return

        if handName == 'RHand':
            self._highNICO_robot.r_indexfingers_x.compliant = False
            self._highNICO_robot.r_indexfingers_x.goal_speed = 10
            self._highNICO_robot.r_indexfingers_x.goal_position = 140
            self._highNICO_robot.r_thumb_x.compliant = False
            self._highNICO_robot.r_thumb_x.goal_speed = 10
            self._highNICO_robot.r_thumb_x.goal_position = 140
            sleep(1)
            self._highNICO_robot.r_indexfingers_x.compliant = True
            self._highNICO_robot.r_thumb_x.compliant = True
        elif handName == 'LHand':
            self._highNICO_robot.l_indexfingers_x.compliant = False
            self._highNICO_robot.l_indexfingers_x.goal_speed = 10
            self._highNICO_robot.l_indexfingers_x.goal_position = 140
            self._highNICO_robot.l_thumb_x.compliant = False
            self._highNICO_robot.l_thumb_x.goal_speed = 10
            self._highNICO_robot.l_thumb_x.goal_position = 140
            sleep(1)
            self._highNICO_robot.l_indexfingers_x.compliant = True
            self._highNICO_robot.l_thumb_x.compliant = True
        else:
            logging.warning('Unknown hand handle: %s' % handName)
            return

    def cleanup(self):
        """
        Cleans up the current connection to the robot. After this you can no longer control the robot
        :return: None
        """
        if self._highNICO_robot is None:
            logging.warning('Cleanup called - but robot is not initialised')
            return

        logging.info('highNICO: Closing robot connection')
        self._highNICO_robot.close()
        self._highNICO_robot = None
        logging.shutdown()

    def __del__(self):
        """
        Destructor
        :return: None
        """
        if self._highNICO_robot is  not None:
            self.cleanup()