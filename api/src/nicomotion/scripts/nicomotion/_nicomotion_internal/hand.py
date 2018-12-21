import logging
import threading
import time

MAX_CUR_FINGER = 100
MAX_CUR_THUMB = 100
CURRENT_PORTS = {"wrist_z": "present_current_port_1",
                 "wrist_x": "present_current_port_2",
                 "thumb_x": "present_current_port_3",
                 "indexfingers_x": "present_current_port_4"}

logger = logging.getLogger(__name__)


def _HAND_compliant(robot):
    """
    Removes the compliant from the hand. This function is used as a callback
    for the timer

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


def _closeHandWithCurrentLimit(board, thumb, indexfingers, percentage):
    for it, pos in enumerate(range(int(indexfingers.present_position),
                                   int(130. * percentage), 5)):
        for retries in range(10):
            success = True
            try:
                if (board.present_current_port_4 > MAX_CUR_FINGER
                        or board.present_current_port_3 > MAX_CUR_THUMB):
                    logger.warning("Reached maximum current - Hand won't " +
                                   "be closed any further")
                    return
                break
            except AttributeError as e:
                if retries == 9:
                    logger.error("Current check failed after 10 retries")
                    success = False
                    raise
                logger.error(
                    "Current check failed - retry {}".format(retries + 1))
        if not success:
            break
        indexfingers.goal_position = pos
        thumb.goal_position = pos
        time.sleep(0.05)
    indexfingers.compliant = True
    thumb.compliant = True


def isHandMotor(jointname):
    """
    Checks whether the given motor belongs to the RH4D hand

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

    """
    Wrist Rotation: ID 23/24, Port Number 1 == wrist_z
    Wrist Flexion: ID 25/26, Port Number 2  == wrist_x
    Thumb Flexion: ID 27/28, Port Number 3  == thumb_x
    Middle finger Flexion: ID 29/30, Port Number 4 == indexfingers_x
    """

    if isHandMotor(jointname):

        if jointname.startswith('r_'):
            board = getattr(robot, "r_virtualhand_x")
        elif jointname.startswith('l_'):
            board = getattr(robot, "l_virtualhand_x")

        # if board != None:
        #    if jointname.endswith("wrist_z"):
        #        return board.present_current_port_1
        #    elif jointname.endswith("wrist_x"):
        #        return board.present_current_port_2
        #    elif jointname.endswith("thumb_x"):
        #        return board.present_current_port_3
        #    elif jointname.endswith("indexfingers_x"):
        #        return board.present_current_port_4

        return getattr(board, CURRENT_PORTS[jointname[2:]])

    logging.warning("{} is not a handjoint".format(jointname))
    return 0


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
        logger.critical('No robot provided')
        return

    if not (0.0 < percentage <= 1.0):
        logger.critical('percentage (%f) out of bounds' % percentage)
        return

    if handName == 'RHand':
        robot.r_indexfingers_x.compliant = False
        robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_indexfingers_x.goal_position = -170.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = -170.0 * percentage
        threading.Timer(1.0, _HAND_compliant, [robot]).start()
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = -170.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = -170.0 * percentage
        threading.Timer(1.0, _HAND_compliant, [robot]).start()
    else:
        logger.warning('Unknown hand handle: %s' % handName)
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
        logger.critical('No robot provided')
        return

    if not (0.0 < percentage <= 1.0):
        logger.critical('percentage (%f) out of bounds' % percentage)
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
        logger.warning('Unknown hand handle: %s' % handName)
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
        logger.critical('No robot provided')
        return

    if not (0.0 < percentage <= 1.0):
        logger.critical('percentage (%f) out of bounds' % percentage)
        return

    if handName == 'RHand':
        robot.r_indexfingers_x.compliant = False
        robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        threading.Thread(target=_closeHandWithCurrentLimit, args=[
                         robot.r_virtualhand_x, robot.r_thumb_x,
                         robot.r_indexfingers_x, percentage]).start()

    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        threading.Thread(target=_closeHandWithCurrentLimit, args=[
                         robot.l_virtualhand_x, robot.l_thumb_x,
                         robot.l_indexfingers_x, percentage]).start()
    else:
        logger.warning('Unknown hand handle: %s' % handName)
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
        logger.critical('No robot provided')
        return

    if not (0.0 < percentage <= 1.0):
        logger.critical('percentage (%f) out of bounds' % percentage)
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
        logger.warning('Unknown hand handle: %s' % handName)
        return
