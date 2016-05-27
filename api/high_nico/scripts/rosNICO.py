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

from highNICO import HighNico
import logging
import argparse
import sys

import rospy
from high_nico.msg import string

class RosNico():
    """
    The RosNico class exposes the functions of HighNico to ROS
    """

    @staticmethod
    def get_config():
        """
        Returns a default config dict
        :return: dict
        """
        return {'log_file': 'NICO.log',
                'robot_motor_file': 'config.json',
                'vrep': False,
                'vrep_host': '127.0.0.1',
                'vrep_port': 19997,
                'vrep_scene': None,
                'rostopic_name': '/nico'
                }

    def __init__(self, config = None):
        """
        RosNico provides HighNico functions over ROS
        :param config: Configuration of the HighNico and RosNico interface
        :type config: dict
        """
        self.robot = None
        if config is None:
            config = get_config()

        # init highNICO
        logging.info('-- Init rosNICO --')
        self.robot = HighNico(motor_config=config['robot_motor_file'], vrep=config['vrep'], vrep_host=config['vrep_host'], vrep_port=config['vrep_port'], vrep_scene=config['vrep_scene'])

        # init ROS
        logging.debug('Init ROS')
        rospy.init_node('ros_nico', anonymous=True)

        # setup subscriber
        logging.debug('Init subscriber')
        rospy.Subscriber('%s/openHand' % config['rostopic_name'], string, self._ROSPY_openHand)
        rospy.Subscriber('%s/closeHand' % config['rostopic_name'], string, self._ROSPY_closeHand)

        # wait for messages
        logging.info('-- All done --')

    def _ROSPY_openHand(self, handName):
        """
        Callback handle for openHand
        :param handName: ROS message
        :type handName: high_nico.msg.string
        :return:
        """
        self.robot.openHand(handName.param1)

    def _ROSPY_closeHand(self, handName):
        """
        Callback handle for closeHand
        :param handName: ROS message
        :type handName: high_nico.msg.string
        :return:
        """
        self.robot.closeHand(handName.param1)

    def __del__(self):
        self.robot.cleanup()

if __name__ == '__main__':
    config = RosNico.get_config()

    # Parse command line
    parser = argparse.ArgumentParser(description='NICO ROS interface')
    parser.add_argument('--log-level', dest='log_level', help='Sets log level. Default: INFO', type=str)
    parser.add_argument('--log-file', dest='log_file', help='Path to log file. Default: %s' % config['log_file'], type=str)
    parser.add_argument('-m', '--motor-file', dest='robot_motor_file', help='Path to robot motor file. Default: %s' % config['robot_motor_file'], type=str)
    parser.add_argument('-v', '--vrep', dest='vrep', help='Connect to VREP rather than to a real robot', action='store_true')
    parser.add_argument('--vrep-host', dest='vrep_host', help='Host of VREP. Default: %s' % config['vrep_host'], type=str)
    parser.add_argument('--vrep-port', dest='vrep_port', help='Port of VREP. Default: %i' % config['vrep_port'], type=int)
    parser.add_argument('--vrep-scene', dest='vrep_scene', help='Scene to load in VREP. Default: %s' % config['vrep_scene'], type=str)
    parser.add_argument('--rostopic-name', dest='rostopic_name', help='Topic name for ROS. Default: %s' % config['rostopic_name'], type=str)

    args = parser.parse_known_args()[0]
    if args.log_file:
        config['log_file'] = args.log_file
    if args.robot_motor_file:
        config['robot_motor_file'] = args.robot_motor_file
    config['vrep'] = args.vrep
    if args.vrep_host:
        config['vrep_host'] = args.vrep_host
    if args.vrep_port:
        config['vrep_port'] = args.vrep_port
    if args.vrep_scene:
        config['vrep_scene'] = args.vrep_scene
    if args.rostopic_name:
        config['rostopic_name'] = args.rostopic_name

    # Set logging setting
    logging_level = logging.INFO
    try:
        logging_level = {
            'DEBUG': logging.DEBUG,
            'INFO': logging.INFO,
            'WARNING': logging.WARNING,
            'CRITICAL': logging.CRITICAL,
            'debug': logging.DEBUG,
            'info': logging.INFO,
            'warning': logging.WARNING,
            'critical': logging.CRITICAL,
        }[args.log_level]
    except:
        sys.stderr.write('LOGGING ERROR: Unknown log level %s\n' % args.log_level)
        pass

    logging.basicConfig(filename=config['log_file'],
                        format='%(asctime)s %(levelname)s at %(funcName)s (%(module)s: %(lineno)d): %(message)s',
                        level=logging_level)
    stdout_handler = logging.StreamHandler(sys.stdout)
    stdout_handler.setLevel(logging_level)
    logging_format = logging.Formatter(
        '%(asctime)s %(levelname)s at %(funcName)s (%(module)s: %(lineno)d): %(message)s')
    stdout_handler.setFormatter(logging_format)
    logging.getLogger().addHandler(stdout_handler)

    ros_connection = RosNico(config)

    rospy.spin()