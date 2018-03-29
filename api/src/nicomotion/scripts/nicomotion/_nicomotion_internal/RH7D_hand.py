import logging
import time
import threading

MAX_CUR=100
CURRENT_PORTS = {"wrist_z":"present_current_port_1",
                 "wrist_y":"present_current_port_2",
                 "wrist_x":"present_current_port_3",
                 "thumb_z":"present_current_port_4",
                 "thumb_x":"present_current_port_5",
                 "indexfinger_x":"present_current_port_6",
                 "middlefingers_x":"present_current_port_7"}


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

def _setGoalPositionWithCurrentLimit(robot, jointname, goal_position, fractionMaxSpeed):
    """
    Approaches the goal position in 5 degree steps. Checks the current on each step
    in order to not damage the hand. Should only be called by a thread.

    :param jointName: Name of the joint
    :type jointName: str
    :param goal_position: Angle (in degree)
    :type goal_position: float
    :param fractionMaxSpeed: Movement speed of joint
    :type fractionMaxSpeed: float
    """
    joint = getattr(robot, jointname)
    if jointname.startswith('r_'):
        board = getattr(robot, "r_virtualhand")
    elif jointname.startswith('l_'):
        board = getattr(robot, "l_virtualhand")

    if board != None and joint != None:
        joint.compliant = False
        joint.goal_speed = 1000.0 * fractionMaxSpeed
        step = 5
        if int(joint.present_position) > int(goal_position):
            step *= -1
        for it,pos in enumerate(range (int(joint.present_position),int(goal_position), step)):
            for retries in range(10):
                success=True
                try:
                    if getattr(board, CURRENT_PORTS[jointname[2:]])>MAX_CUR:
                        logging.warning("Reached maximum current - Stopping movement of {}".format(jointname))
                        return
                    break
                except AttributeError as e:
                    if retries==9:
                        logging.warning("Current check failed after 10 retries")
                        success=False
                        raise
                    logging.warning("Current check failed - retry {}".format(retries+1))
            if not success:
                break
            joint.goal_position=pos
            time.sleep(.01)
        time.sleep(1)
        joint.compliant = True

def _move(motor, position, fractionMaxSpeed):
    """
    Moves motor to given position.

    :param motor: Motor object
    :type motor: pypot.robot.motor
    :param position: goal position/angle
    :type position: float
    :param fractionMaxSpeed: Percentage of goal speed at which the motor should operate [0.0, 1.0]
    :type position: float
    """
    motor.compliant = False
    motor.goal_speed = 1000.0 * fractionMaxSpeed
    motor.goal_position = position

def isHandMotor(jointname):
    """
    Checks whether the given motor belongs to the RH7D hand

    :param jointname: Name of the motor
    :type jointname: str
    :return: True if motor is a hand motor, False else
    :rtype: boolean
    """
    if jointname[2:] in CURRENT_PORTS.keys():
        return True
    return False

def getPresentCurrent(robot, jointname):
    """
    Returns the current reading for the given joint from the hand's mainboard.
    (Current readings are not stored in the motors themselves)

    :param jointName: Name of the joint
    :type jointName: str
    :return: Current of the joint
    :rtype: float
    """
    if isHandMotor(jointname):

        if jointname.startswith('r_'):
            board = getattr(robot, "r_virtualhand")
        elif jointname.startswith('l_'):
            board = getattr(robot, "l_virtualhand")

        if board != None:
            return getattr(board, CURRENT_PORTS[jointname[2:]])

    logging.warning("{} is not a handjoint".format(jointname))
    return 0

def setAngle(robot, jointname, goal_position, fractionMaxSpeed):
    """
    Sets the angle of a given hand joint to the given goal position. (aborts if current is too high)

    :param jointName: Name of the joint
    :type jointName: str
    :param goal_position: Angle (in degree)
    :type goal_position: float
    :param fractionMaxSpeed: Movement speed of joint
    :type fractionMaxSpeed: float
    """
    if isHandMotor(jointname):
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, jointname, goal_position, fractionMaxSpeed]).start()
    else:
        logging.warning("{} is not a handjoint".format(jointname))

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
        for motor in ["r_thumb_x","r_indexfinger_x","r_middlefingers_x","r_thumb_z"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, -180,fractionMaxSpeed]).start()
        threading.Timer(2.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        for motor in ["l_thumb_x","l_indexfinger_x","l_middlefingers_x","l_thumb_z"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, -180,fractionMaxSpeed]).start()
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
        for motor in ["r_indexfinger_x","r_middlefingers_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 180,fractionMaxSpeed]).start()
        for motor in ["r_thumb_z","r_thumb_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, -180,fractionMaxSpeed]).start()
    elif handName == 'LHand':
        for motor in ["l_indexfinger_x","l_middlefingers_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 180,fractionMaxSpeed]).start()
        for motor in ["l_thumb_z","l_thumb_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, -180,fractionMaxSpeed]).start()
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
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_indexfinger_x", -180, fractionMaxSpeed]).start()
        for motor in ["r_middlefingers_x","r_thumb_z"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_thumb_x", 90,.9*fractionMaxSpeed]).start()
    elif handName == 'LHand':
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_indexfinger_x", -180,fractionMaxSpeed]).start()
        for motor in ["l_middlefingers_x","l_thumb_z"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_thumb_x", 90,.9*fractionMaxSpeed]).start()
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
        for motor in ["r_indexfinger_x","r_thumb_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 0,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_thumb_z", 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_middlefingers_x", -180,fractionMaxSpeed]).start()
    elif handName == 'LHand':
        for motor in ["l_indexfinger_x","l_thumb_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 0,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_thumb_z", 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_middlefingers_x", -180,fractionMaxSpeed]).start()
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
        for motor in ["r_indexfinger_x","robot.r_thumb_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 0,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_thumb_z", 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_middlefingers_x", 180,fractionMaxSpeed]).start()
    elif handName == 'LHand':
        for motor in ["l_indexfinger_x","l_thumb_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 0,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_thumb_z", 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_middlefingers_x", 180,fractionMaxSpeed]).start()
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
        for motor in ["r_indexfinger_x","r_thumb_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 30,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_thumb_z", -180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_middlefingers_x", 180,fractionMaxSpeed]).start()
    elif handName == 'LHand':
        for motor in ["l_indexfinger_x","l_thumb_x"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 30,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_thumb_z", -180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_middlefingers_x", 180,fractionMaxSpeed]).start()
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
        for motor in ["r_middlefingers_x","r_thumb_z"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_thumb_z", -180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_middlefingers_x", 180,fractionMaxSpeed]).start()
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
        for motor in ["r_indexfinger_x","r_middlefingers_x","r_thumb_z"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "r_thumb_x", 90,.5*fractionMaxSpeed]).start()
        threading.Timer(5.0, _HAND_compliant, [robot]).start()

    elif handName == 'LHand':
        for motor in ["l_indexfinger_x","l_middlefingers_x","l_thumb_z"]:
            threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, motor, 180,fractionMaxSpeed]).start()
        threading.Thread(target=_setGoalPositionWithCurrentLimit, args=[robot, "l_thumb_x", 90,.5*fractionMaxSpeed]).start()
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
