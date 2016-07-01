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
        robot.r_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_indexfingers_x.goal_position = 130.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = 130.0 * percentage
        time.sleep(1)
        robot.r_indexfingers_x.compliant = True
        robot.r_thumb_x.compliant = True
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = 130.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = 130.0 * percentage
        time.sleep(1)
        robot.l_indexfingers_x.compliant = True
        robot.l_thumb_x.compliant = True
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

    #TODO: Correct this once the new naming convention is at the scene
    if handName == 'RHand':
        robot.finger_segment_2__right_palm_11.compliant = False
        robot.finger_segment_2__finger_segment_12.compliant = False
        robot.finger_segment_1__fingertip_12.compliant = False
        robot.finger_segment_2__right_palm_12.compliant = False
        robot.finger_segment_2__finger_segment_11.compliant = False
        robot.finger_segment_1__fingertip_11.compliant = False
        robot.finger_segment_2__right_palm_13.compliant = False
        robot.finger_segment_2__finger_segment_13.compliant = False
        robot.finger_segment_1__fingertip_13.compliant = False

        robot.finger_segment_2__right_palm_11.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_12.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_12.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__right_palm_12.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_11.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_11.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__right_palm_13.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_13.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_13.goal_speed = 1000.0 * fractionMaxSpeed
        
        robot.finger_segment_2__right_palm_11.goal_position = 0.0 * percentage
        robot.finger_segment_2__finger_segment_12.goal_position = 0.0 * percentage
        robot.finger_segment_1__fingertip_12.goal_position = 0.0 * percentage
        robot.finger_segment_2__right_palm_12.goal_position = 0.0 * percentage
        robot.finger_segment_2__finger_segment_11.goal_position = 0.0 * percentage
        robot.finger_segment_1__fingertip_11.goal_position = 0.0 * percentage
        robot.finger_segment_2__right_palm_13.goal_position = 0.0 * percentage
        robot.finger_segment_2__finger_segment_13.goal_position = 0.0 * percentage
        robot.finger_segment_1__fingertip_13.goal_position = 0.0 * percentage

    elif handName == 'LHand':
        robot.finger_segment_2__left_palm_11.compliant = False
        robot.finger_segment_2__finger_segment_14.compliant = False
        robot.finger_segment_1__fingertip_14.compliant = False
        robot.finger_segment_2__left_palm_12.compliant = False
        robot.finger_segment_2__finger_segment_15.compliant = False
        robot.finger_segment_1__fingertip_15.compliant = False
        robot.finger_segment_2__left_palm_13.compliant = False
        robot.finger_segment_2__finger_segment_16.compliant = False
        robot.finger_segment_1__fingertip_16.compliant = False

        robot.finger_segment_2__left_palm_11.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_14.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_14.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__left_palm_12.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_15.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_15.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__left_palm_13.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_16.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_16.goal_speed = 1000.0 * fractionMaxSpeed

        robot.finger_segment_2__left_palm_11.goal_position = 0.0 * percentage
        robot.finger_segment_2__finger_segment_14.goal_position = 0.0 * percentage
        robot.finger_segment_1__fingertip_14.goal_position = 0.0 * percentage
        robot.finger_segment_2__left_palm_12.goal_position = 0.0 * percentage
        robot.finger_segment_2__finger_segment_15.goal_position = 0.0 * percentage
        robot.finger_segment_1__fingertip_15.goal_position = 0.0 * percentage
        robot.finger_segment_2__left_palm_13.goal_position = 0.0 * percentage
        robot.finger_segment_2__finger_segment_16.goal_position = 0.0 * percentage
        robot.finger_segment_1__fingertip_16.goal_position = 0.0 * percentage
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
        robot.r_indexfingers_x.goal_position = -130.0 * percentage
        robot.r_thumb_x.compliant = False
        robot.r_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.r_thumb_x.goal_position = -130.0 * percentage
        time.sleep(1)
        robot.r_indexfingers_x.compliant = True
        robot.r_thumb_x.compliant = True
    elif handName == 'LHand':
        robot.l_indexfingers_x.compliant = False
        robot.l_indexfingers_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_indexfingers_x.goal_position = -130.0 * percentage
        robot.l_thumb_x.compliant = False
        robot.l_thumb_x.goal_speed = 1000.0 * fractionMaxSpeed
        robot.l_thumb_x.goal_position = -130.0 * percentage
        time.sleep(1)
        robot.l_indexfingers_x.compliant = True
        robot.l_thumb_x.compliant = True
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

    # TODO: Correct this once the new naming convention is at the scene
    if handName == 'RHand':
        robot.finger_segment_2__right_palm_11.compliant = False
        robot.finger_segment_2__finger_segment_12.compliant = False
        robot.finger_segment_1__fingertip_12.compliant = False
        robot.finger_segment_2__right_palm_12.compliant = False
        robot.finger_segment_2__finger_segment_11.compliant = False
        robot.finger_segment_1__fingertip_11.compliant = False
        robot.finger_segment_2__right_palm_13.compliant = False
        robot.finger_segment_2__finger_segment_13.compliant = False
        robot.finger_segment_1__fingertip_13.compliant = False

        robot.finger_segment_2__right_palm_11.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_12.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_12.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__right_palm_12.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_11.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_11.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__right_palm_13.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_13.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_13.goal_speed = 1000.0 * fractionMaxSpeed

        robot.finger_segment_2__right_palm_11.goal_position = 30.0 * percentage
        robot.finger_segment_2__finger_segment_12.goal_position = 30.0 * percentage
        robot.finger_segment_1__fingertip_12.goal_position = 30.0 * percentage
        robot.finger_segment_2__right_palm_12.goal_position = 30.0 * percentage
        robot.finger_segment_2__finger_segment_11.goal_position = 30.0 * percentage
        robot.finger_segment_1__fingertip_11.goal_position = 30.0 * percentage
        robot.finger_segment_2__right_palm_13.goal_position = 30.0 * percentage
        robot.finger_segment_2__finger_segment_13.goal_position = 30.0 * percentage
        robot.finger_segment_1__fingertip_13.goal_position = 30.0 * percentage

    elif handName == 'LHand':
        robot.finger_segment_2__left_palm_11.compliant = False
        robot.finger_segment_2__finger_segment_14.compliant = False
        robot.finger_segment_1__fingertip_14.compliant = False
        robot.finger_segment_2__left_palm_12.compliant = False
        robot.finger_segment_2__finger_segment_15.compliant = False
        robot.finger_segment_1__fingertip_15.compliant = False
        robot.finger_segment_2__left_palm_13.compliant = False
        robot.finger_segment_2__finger_segment_16.compliant = False
        robot.finger_segment_1__fingertip_16.compliant = False

        robot.finger_segment_2__left_palm_11.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_14.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_14.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__left_palm_12.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_15.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_15.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__left_palm_13.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_2__finger_segment_16.goal_speed = 1000.0 * fractionMaxSpeed
        robot.finger_segment_1__fingertip_16.goal_speed = 1000.0 * fractionMaxSpeed

        robot.finger_segment_2__left_palm_11.goal_position = 30.0 * percentage
        robot.finger_segment_2__finger_segment_14.goal_position = 30.0 * percentage
        robot.finger_segment_1__fingertip_14.goal_position = 30.0 * percentage
        robot.finger_segment_2__left_palm_12.goal_position = 30.0 * percentage
        robot.finger_segment_2__finger_segment_15.goal_position = 30.0 * percentage
        robot.finger_segment_1__fingertip_15.goal_position = 30.0 * percentage
        robot.finger_segment_2__left_palm_13.goal_position = 30.0 * percentage
        robot.finger_segment_2__finger_segment_16.goal_position = 30.0 * percentage
        robot.finger_segment_1__fingertip_16.goal_position = 30.0 * percentage
    else:
        logging.warning('Unknown hand handle: %s' % handName)
        return