import logging
import time
import threading

def _HAND_compliant(robot):
    """
    Removes the compliant from the hand. This function is used as a callback for the timer

    :param robot: The robot
    :type robot: pypot.robot
    """
    if hasattr(robot, 'r_indexfinger_x'):
        robot.r_indexfinger_x.compliant = True
    if hasattr(robot, 'r_middlefingers_x'):
        robot.r_middlefingers_x.compliant = True
    if hasattr(robot, 'r_thumb_x'):
        robot.r_thumb_x.compliant = True
    if hasattr(robot, 'r_thumb_z'):
        robot.r_thumb_z.compliant = True

    if hasattr(robot, 'l_indexfinger_x'):
        robot.l_indexfinger_x.compliant = True
    if hasattr(robot, 'l_middlefingers_x'):
        robot.l_middlefingers_x.compliant = True
    if hasattr(robot, 'l_thumb_x'):
        robot.l_thumb_x.compliant = True
    if hasattr(robot, 'l_thumb_z'):
        robot.l_thumb_z.compliant = True

def openHand(robot, handName, fractionMaxSpeed=1.0):
    """
    Opens the specified hand. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
    :type fractionMaxSpeed: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if handName == 'RHand':
        for motor in [robot.r_thumb_x,robot.r_indexfinger_x,robot.r_middlefingers_x,robot.r_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = -180
        threading.Timer(2.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_thumb_x,robot.l_indexfinger_x,robot.l_middlefingers_x,robot.l_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = -180
        threading.Timer(2.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

# def openHandVREP(robot, handName, fractionMaxSpeed=1.0, percentage=1.0):
#     """
#     Opens the specified hand. handName can be 'RHand' or 'LHand'
#
#     This function does the conversion to the V-REP simulator
#
#     :param robot: Robot object
#     :type robot: pypot.robot
#     :param handName: Name of the hand (RHand, LHand)
#     :type handName: str
#     :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
#     :type fractionMaxSpeed: float
#     :param percentage: Percentage hand should open. 0.0 < percentage <= 1.0
#     :type percentage: float
#     :return: None
#     """
#     if robot is None:
#         logging.critical('No robot provided')
#         return
#
#     if not (0.0 < percentage <= 1.0):
#         logging.critical('percentage (%f) out of bounds' % percentage)
#         return
#
#     if handName == 'RHand':
#         robot.r_indexfingers_x.compliant = False
#         robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
#         robot.r_indexfingers_x.goal_position = 0.0 * percentage
#         robot.r_thumb_x.compliant = False
#         robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
#         robot.r_thumb_x.goal_position = 0.0 * percentage
#     elif handName == 'LHand':
#         robot.l_indexfingers_x.compliant = False
#         robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
#         robot.l_indexfingers_x.goal_position = 0.0 * percentage
#         robot.l_thumb_x.compliant = False
#         robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
#         robot.l_thumb_x.goal_position = 0.0 * percentage
#     else:
#         logging.warning('Unknown hand handle: %s' % handName)
#         return

def thumbsUp(robot, handName, fractionMaxSpeed=1.0):
    """
    Gives a thumbs up with the specified hand. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
    :type fractionMaxSpeed: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if handName == 'RHand':
        for motor in [robot.r_indexfinger_x,robot.r_middlefingers_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0
            motor.goal_position = 180
        for motor in [robot.r_thumb_z,robot.r_thumb_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0
            motor.goal_position = -180
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_middlefingers_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0
            motor.goal_position = 180
        for motor in [robot.l_thumb_z,robot.l_thumb_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0
            motor.goal_position = -180
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def pointAt(robot, handName, fractionMaxSpeed=1.0):
    """
    Sticks the indexfinger out to point at something. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
    :type fractionMaxSpeed: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if handName == 'RHand':
        robot.r_indexfinger_x.compliant = False
        robot.r_indexfinger_x.goal_speed = 1000.0
        robot.r_indexfinger_x.goal_position = -180
        for motor in [robot.r_middlefingers_x,robot.r_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 180
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 500.0
        robot.r_thumb_x.goal_position = 90
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        robot.l_indexfinger_x.compliant = False
        robot.l_indexfinger_x.goal_speed = 1000.0
        robot.l_indexfinger_x.goal_position = -180
        for motor in [robot.l_middlefingers_x,robot.l_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 180
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 500.0
        robot.l_thumb_x.goal_position = 90
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def okSign(robot, handName, fractionMaxSpeed=1.0):
    """
    Performs the 'OK' hand signal that divers use. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
    :type fractionMaxSpeed: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if handName == 'RHand':
        for motor in [robot.r_indexfinger_x,robot.r_thumb_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 25
        robot.r_thumb_z.compliant = False
        robot.r_thumb_z.goal_speed = 1000.0
        robot.r_thumb_z.goal_position = 180
        robot.r_middlefingers_x.compliant = False
        robot.r_middlefingers_x.goal_speed = 1000.0
        robot.r_middlefingers_x.goal_position = -180
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_thumb_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 25
        robot.l_thumb_z.compliant = False
        robot.l_thumb_z.goal_speed = 1000.0
        robot.l_thumb_z.goal_position = 180
        robot.l_middlefingers_x.compliant = False
        robot.l_middlefingers_x.goal_speed = 1000.0
        robot.l_middlefingers_x.goal_position = -180
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def pinchToIndex(robot, handName, fractionMaxSpeed=1.0):
    """
    Pinches thumb and index finger together. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
    :type fractionMaxSpeed: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if handName == 'RHand':
        for motor in [robot.r_indexfinger_x,robot.r_thumb_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 25
        for motor in [robot.r_middlefingers_x,robot.r_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 180
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_thumb_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 25
        for motor in [robot.l_middlefingers_x,robot.l_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 180
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def keyGrip(robot, handName, fractionMaxSpeed=1.0):
    """
    Performs a grip that can hold a key card. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
    :type fractionMaxSpeed: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if handName == 'RHand':
        for motor in [robot.r_indexfinger_x,robot.r_thumb_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 45
        robot.r_thumb_z.compliant = False
        robot.r_thumb_z.goal_speed = 1000.0
        robot.r_thumb_z.goal_position = -180
        robot.r_middlefingers_x.compliant = False
        robot.r_middlefingers_x.goal_speed = 1000.0
        robot.r_middlefingers_x.goal_position = 180
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_thumb_x]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 45
        robot.l_thumb_z.compliant = False
        robot.l_thumb_z.goal_speed = 1000.0
        robot.l_thumb_z.goal_position = -180
        robot.l_middlefingers_x.compliant = False
        robot.l_middlefingers_x.goal_speed = 1000.0
        robot.l_middlefingers_x.goal_position = -180
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def pencilGrip(robot, handName, fractionMaxSpeed=1.0):
    """
    Performs a grip able to hold a (thick) pencil. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
    :type fractionMaxSpeed: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if handName == 'RHand':
        for motor in [robot.r_middlefingers_x,robot.r_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 180
        robot.r_indexfinger_x.compliant = False
        robot.r_indexfinger_x.goal_speed = 400.0 * fractionMaxSpeed
        robot.r_indexfinger_x.goal_position = 90
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 600.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = 90
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_middlefingers_x,robot.l_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0 * fractionMaxSpeed
            motor.goal_position = 180
        robot.l_indexfinger_x.compliant = False
        robot.l_indexfinger_x.goal_speed = 400.0 * fractionMaxSpeed
        robot.l_indexfinger_x.goal_position = 90
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 600.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = 90
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def closeHand(robot, handName, fractionMaxSpeed=1.0):
    """
    Closes the specified hand. handName can be 'RHand' or 'LHand'

    :param robot: Robot object
    :type robot: pypot.robot
    :param handName: Name of the hand (RHand, LHand)
    :type handName: str
    :param fractionMaxSpeed: Speed at which hand should close. Default: 1.0
    :type fractionMaxSpeed: float
    :return: None
    """
    if robot is None:
        logging.critical('No robot provided')
        return

    if handName == 'RHand':
        for motor in [robot.r_indexfinger_x,robot.r_middlefingers_x,robot.r_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0
            motor.goal_position = 180
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 500.0
        robot.r_thumb_x.goal_position = 90
        threading.Timer(5.0, _HAND_compliant, [robot]).start()

    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_middlefingers_x,robot.l_thumb_z]:
            motor.compliant = False
            motor.goal_speed = 1000.0
            motor.goal_position = 180
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 500.0
        robot.l_thumb_x.goal_position = 90
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return


# def closeHandVREP(robot, handName, fractionMaxSpeed=1.0, percentage=1.0):
#     """
#     Opens the specified hand. handName can be 'RHand' or 'LHand'
#
#     This function does the conversion to the V-REP simulator
#
#     :param robot: Robot object
#     :type robot: pypot.robot
#     :param handName: Name of the hand (RHand, LHand)
#     :type handName: str
#     :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
#     :type fractionMaxSpeed: float
#     :param percentage: Percentage hand should open. 0.0 < percentage <= 1.0
#     :type percentage: float
#     :return: None
#     """
#     if robot is None:
#         logging.critical('No robot provided')
#         return
#
#     if not (0.0 < percentage <= 1.0):
#         logging.critical('percentage (%f) out of bounds' % percentage)
#         return
#
#     if handName == 'RHand':
#         robot.r_indexfingers_x.compliant = False
#         robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
#         robot.r_indexfingers_x.goal_position = -30.0 * percentage
#         robot.r_thumb_x.compliant = False
#         robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
#         robot.r_thumb_x.goal_position = -30.0 * percentage
#     elif handName == 'LHand':
#         robot.l_indexfingers_x.compliant = False
#         robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
#         robot.l_indexfingers_x.goal_position = -30.0 * percentage
#         robot.l_thumb_x.compliant = False
#         robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
#         robot.l_thumb_x.goal_position = -30.0 * percentage
#     else:
#         logging.warning('Unknown hand handle: %s' % handName)
#         return
