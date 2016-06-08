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

    def __init__(self, motorConfig='config.json', vrep=False, vrepHost='127.0.0.1', vrepPort=19997, vrepScene=None):
        """
        HighNico is a high level interface to the NICO robot,
        :param motorConfig: motor config file (JSON format)
        :type motorConfig: str
        :param vrep: If set to true VREP will be used instead of real robot
        :type vrep: bool
        :param vrepHost: Network address of VREP
        :type vrepHost: str
        :param vrepPort: Port of VREP
        :type vrepPort: int
        :param vrepScene: Scene to load. Set to None to use current scene
        :type vrepScene: str
        """
        self._highNicoRobot = None

        with open(motorConfig, 'r') as config_file:
            config = json.load(config_file)

        if vrep:
            logging.info('highNICO: Using VREP')
            self._highNicoRobot = pypot.vrep.from_vrep(config, vrepHost, vrepPort, vrepScene)
        else:
            logging.info('highNICO: Using robot')
            self._highNicoRobot = pypot.robot.from_config(config)

    def openHand(self, handName):
        """
        Opens the specified hand. handName can be 'RHand' or 'LHand'
        :param handName: Name of the hand (RHand, LHand)
        :return: None
        """
        if self._highNicoRobot is None:
            logging.critical('NICO not initialised')
            return

        if handName == 'RHand':
            self._highNicoRobot.r_indexfingers_x.compliant = False
            self._highNicoRobot.r_indexfingers_x.goal_speed = 10
            self._highNicoRobot.r_indexfingers_x.goal_position = -130
            self._highNicoRobot.r_thumb_x.compliant = False
            self._highNicoRobot.r_thumb_x.goal_speed = 10
            self._highNicoRobot.r_thumb_x.goal_position = -130
            sleep(1)
            self._highNicoRobot.r_indexfingers_x.compliant = True
            self._highNicoRobot.r_thumb_x.compliant = True
        elif handName == 'LHand':
            self._highNicoRobot.l_indexfingers_x.compliant = False
            self._highNicoRobot.l_indexfingers_x.goal_speed = 10
            self._highNicoRobot.l_indexfingers_x.goal_position = -130
            self._highNicoRobot.l_thumb_x.compliant = False
            self._highNicoRobot.l_thumb_x.goal_speed = 10
            self._highNicoRobot.l_thumb_x.goal_position = -130
            sleep(1)
            self._highNicoRobot.l_indexfingers_x.compliant = True
            self._highNicoRobot.l_thumb_x.compliant = True
        else:
            logging.warning('Unknown hand handle: %s' % handName)
            return

    def closeHand(self, handName):
        """
        Closes the specified hand. handName can be 'RHand' or 'LHand'
        :param handName: Name of the hand (RHand, LHand)
        :return: None
        """
        if self._highNicoRobot is None:
            logging.critical('NICO not initialised')
            return

        if handName == 'RHand':
            self._highNicoRobot.r_indexfingers_x.compliant = False
            self._highNicoRobot.r_indexfingers_x.goal_speed = 10
            self._highNicoRobot.r_indexfingers_x.goal_position = 130
            self._highNicoRobot.r_thumb_x.compliant = False
            self._highNicoRobot.r_thumb_x.goal_speed = 10
            self._highNicoRobot.r_thumb_x.goal_position = 130
            print(self.label)
            sleep(1)
            self._highNicoRobot.r_indexfingers_x.compliant = True
            self._highNicoRobot.r_thumb_x.compliant = True
        elif handName == 'LHand':
            self._highNicoRobot.l_indexfingers_x.compliant = False
            self._highNicoRobot.l_indexfingers_x.goal_speed = 10
            self._highNicoRobot.l_indexfingers_x.goal_position = 130
            self._highNicoRobot.l_thumb_x.compliant = False
            self._highNicoRobot.l_thumb_x.goal_speed = 10
            self._highNicoRobot.l_thumb_x.goal_position = 130
            sleep(1)
            self._highNicoRobot.l_indexfingers_x.compliant = True
            self._highNicoRobot.l_thumb_x.compliant = True
        else:
            logging.warning('Unknown hand handle: %s' % handName)
            return

    def cleanup(self):
        """
        Cleans up the current connection to the robot. After this you can no longer control the robot
        :return: None
        """
        if self._highNicoRobot is None:
            logging.warning('Cleanup called - but robot is not initialised')
            return

        logging.info('highNICO: Closing robot connection')
        self._highNicoRobot.close()
        self._highNicoRobot = None
        logging.shutdown()

    def __del__(self):
        """
        Destructor
        :return: None
        """
        if self._highNicoRobot is  not None:
            self.cleanup()