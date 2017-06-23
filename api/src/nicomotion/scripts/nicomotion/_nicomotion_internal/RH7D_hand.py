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

def _move(motor, position, fractionMaxSpeed):
    motor.compliant = False
    motor.goal_speed = 1000.0 * fractionMaxSpeed
    motor.goal_position = position

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
            _move(motor, -180, fractionMaxSpeed)
        threading.Timer(2.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_thumb_x,robot.l_indexfinger_x,robot.l_middlefingers_x,robot.l_thumb_z]:
            _move(motor, -180, fractionMaxSpeed)
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
            _move(motor,180,fractionMaxSpeed)
        for motor in [robot.r_thumb_z,robot.r_thumb_x]:
            _move(motor,-180,fractionMaxSpeed)
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_middlefingers_x]:
            _move(motor,180,fractionMaxSpeed)
        for motor in [robot.l_thumb_z,robot.l_thumb_x]:
            _move(motor,-180,fractionMaxSpeed)
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
        _move(robot.r_indexfinger_x,-180,fractionMaxSpeed)
        for motor in [robot.r_middlefingers_x,robot.r_thumb_z]:
            _move(motor,180,fractionMaxSpeed)
        _move(robot.r_thumb_x,90,0.9*fractionMaxSpeed)
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        _move(robot.l_indexfinger_x,-180,fractionMaxSpeed)
        for motor in [robot.l_middlefingers_x,robot.l_thumb_z]:
            _move(motor,180,fractionMaxSpeed)
        _move(robot.r_thumb_x,90,0.9*fractionMaxSpeed)
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
            _move(motor,0,fractionMaxSpeed)
        _move(robot.r_thumb_z,180,fractionMaxSpeed)
        _move(robot.r_middlefingers_x,-180,fractionMaxSpeed)
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_thumb_x]:
            _move(motor,0,fractionMaxSpeed)
        _move(robot.l_thumb_z,180,fractionMaxSpeed)
        _move(robot.l_middlefingers_x,-180,fractionMaxSpeed)
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
            _move(motor,0,fractionMaxSpeed)
        _move(robot.r_thumb_z,180,fractionMaxSpeed)
        _move(robot.r_middlefingers_x,180,fractionMaxSpeed)
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_thumb_x]:
            _move(motor,0,fractionMaxSpeed)
        _move(robot.l_thumb_z,180,fractionMaxSpeed)
        _move(robot.l_middlefingers_x,180,fractionMaxSpeed)
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return

def keyGrip(robot, handName, fractionMaxSpeed=1.0):
    """
    Performs a grip that can hold a key card. handName can be 'RHand' or 'LHand'

    :param robot: Robot object6
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
            _move(motor,30,fractionMaxSpeed)
        _move(robot.r_thumb_z,-180,fractionMaxSpeed)
        _move(robot.r_middlefingers_x,180,fractionMaxSpeed)
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_thumb_x]:
            _move(motor,30,fractionMaxSpeed)
        _move(robot.l_thumb_z,-180,fractionMaxSpeed)
        _move(robot.l_middlefingers_x,180,fractionMaxSpeed)
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
            _move(motor,180,fractionMaxSpeed)
        _move(robot.r_indexfinger_x,90,0.3*fractionMaxSpeed)
        _move(robot.r_thumb_x,90,0.6*fractionMaxSpeed)
        threading.Timer(5.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in [robot.l_middlefingers_x,robot.l_thumb_z]:
            _move(motor,180,fractionMaxSpeed)
        _move(robot.l_indexfinger_x,90,0.3*fractionMaxSpeed)
        _move(robot.l_thumb_x,90,0.6*fractionMaxSpeed)
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
            _move(motor,180,fractionMaxSpeed)
        _move(robot.r_thumb_x,90,0.5*fractionMaxSpeed)
        threading.Timer(5.0, _HAND_compliant, [robot]).start()

    elif handName == 'LHand':
        for motor in [robot.l_indexfinger_x,robot.l_middlefingers_x,robot.l_thumb_z]:
            _move(motor,180,fractionMaxSpeed)
        _move(robot.r_thumb_x,90,0.4*fractionMaxSpeed)
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
