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
import json
import time

import pypot.robot
import pypot.vrep

import _internal.hand

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

    def openHand(self, handName, fractionMaxSpeed=1.0, percentage=1.0):
        """
        Opens the specified hand. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
        :type fractionMaxSpeed: float
        :param percentage: Percentage hand should open. 0.0 < percentage <= 1.0
        :type percentage: float
        :return: None
        """
        _internal.hand.closeHand(self._highNicoRobot, handName, fractionMaxSpeed, percentage)

    def closeHand(self, handName, fractionMaxSpeed=1.0, percentage=1.0):
        """
        Closes the specified hand. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should close. Default: 1.0
        :type fractionMaxSpeed: float
        :param percentage: Percentage hand should open. 0.0 < percentage <= 1.0
        :type percentage: float
        :return: None
        """
        _internal.hand.openHand(self._highNicoRobot, handName, fractionMaxSpeed, percentage)

    def moveWrist(self, handName, x, z, fractionMaxSpeed=1.0):
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
        :param fractionMaxSpeed: Speed at which hand should close. Default: 1.0
        :type fractionMaxSpeed: float
        :return: none
        """
        _internal.hand.moveWrist(self._highNicoRobot, handName, x, z, fractionMaxSpeed)

    def enableForceControl(self, goalForce = 500):
        """
        Enables force control for all joints which support this feature

        :param goalForce: Goal force (0-2000)
        :type goalForce: int
        :return: None
        """
        for motor in self._highNicoRobot.motors:
            if hasattr(motor, 'force_control_enable'):
                motor.force_control_enable = True
                motor.goal_force = goalForce

    def disableForceControl(self):
        """
        Disables force control for all joints which support this feature

        :return: None
        """
        for motor in self._highNicoRobot.motors:
            if hasattr(motor, 'force_control_enable'):
                motor.force_control_enable = False

    def enableForceControlSingleJoin(self, jointName, goalForce):
        """
        Enables force control for a single joint

        :param jointName: Name of the joint
        :type jointName: str
        :param goalForce: Goal force (0-2000)
        :type goalForce: int
        :return: None
        """
        if hasattr(self._highNicoRobot, jointName):
            motor = getattr(self._highNicoRobot, jointName)
            if hasattr(motor, 'force_control_enable'):
                motor.force_control_enable = True
                motor.goal_force = goalForce
            else:
                logging.warning('Joint %s has no force control' % jointName)
        else:
            logging.warning('No joint "%s" found' % jointName)
            return

    def disableForceControlSingleJoin(self, jointName):
        """
        Disables force control for a single joint

        :param jointName: Name of the joint
        :type jointName: str
        :return: None
        """
        if hasattr(self._highNicoRobot, jointName):
            motor = getattr(self._highNicoRobot, jointName)
            if hasattr(motor, 'force_control_enable'):
                motor.force_control_enable = False
            else:
                logging.warning('Joint %s has no force control' % jointName)
        else:
            logging.warning('No joint "%s" found' % jointName)
            return

    def setAngles(self, jointName, angle, fractionMaxSpeed):
        """
        Sets the angle of a given joint to an angle (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :param angle: Angle (in degree)
        :type angle: float
        :param fractionMaxSpeed: Movement speed of joint
        :type fractionMaxSpeed: float
        :return: None
        """
        if hasattr(self._highNicoRobot, jointName):
            motor = getattr(self._highNicoRobot, jointName)
            motor.compliant = False
            motor.goal_speed = 10.0 * fractionMaxSpeed
            motor.goal_position = angle
            time.sleep(1)
            motor.compliant = True
        else:
            logging.warning('No joint "%s" found' % jointName)
            return

    def changeAngles(self, name, change, fractionMaxSpeed):
        """
        Changes the angle of a given joint by an angle (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :param angle: Angle (in degree)
        :type angle: float
        :param fractionMaxSpeed: Movement speed of joint
        :type fractionMaxSpeed: float
        :return: None
        """
        if hasattr(self._highNicoRobot, jointName):
            motor = getattr(self._highNicoRobot, jointName)
            motor.compliant = False
            motor.goal_speed = 10.0 * fractionMaxSpeed
            motor.goal_position = change + motor.present_position
            time.sleep(1)
            motor.compliant = True
        else:
            logging.warning('No joint "%s" found' % jointName)
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