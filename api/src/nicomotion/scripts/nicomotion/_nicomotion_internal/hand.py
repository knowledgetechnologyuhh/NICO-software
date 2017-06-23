import logging
import time
import threading

def _HAND_compliant(robot):
    """
    Removes the compliant from the hand. This function is used as a callback for the timer

    :param robot: The robot
    :type robot: pypot.robot
    """
    if hasattr(robot, 'r_indexfingers_x'):
        robot.r_indexfingers_x.compliant = True

    if hasattr(robot, 'r_thumb_x'):
        robot.r_thumb_x.compliant = True

    if hasattr(robot, 'l_indexfingers_x'):
        robot.l_indexfingers_x.compliant = True

    if hasattr(robot, 'l_thumb_x'):
        robot.l_thumb_x.compliant = True

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
        robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_indexfingers_x.goal_position = -130.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = -130.0 * percentage
        threading.Timer(1.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = -130.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = -130.0 * percentage
        threading.Timer(1.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def openHandVREP(robot, handName, fractionMaxSpeed=1.0, percentage=1.0):
    """
    Opens the specified hand. handName can be 'RHand' or 'LHand'

    This function does the conversion to the V-REP simulator

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
        robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_indexfingers_x.goal_position = 0.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = 0.0 * percentage
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = 0.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = 0.0 * percentage
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
        robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_indexfingers_x.goal_position = 130.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = 130.0 * percentage
        threading.Timer(1.0, _HAND_compliant, [robot]).start()

    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = 130.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = 130.0 * percentage
        threading.Timer(1.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return


def closeHandVREP(robot, handName, fractionMaxSpeed=1.0, percentage=1.0):
    """
    Opens the specified hand. handName can be 'RHand' or 'LHand'

    This function does the conversion to the V-REP simulator

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
        robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_indexfingers_x.goal_position = -30.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = -30.0 * percentage
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = -30.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = -30.0 * percentage
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return
