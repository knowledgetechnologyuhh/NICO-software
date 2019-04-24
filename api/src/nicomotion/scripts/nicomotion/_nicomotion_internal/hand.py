import logging
import threading
import time
from abc import ABCMeta, abstractmethod, abstractproperty
from threading import Semaphore


class AbstractHand(object):
    """Abstract hand class to represent Seed Robotics hands."""
    __metaclass__ = ABCMeta

    @abstractproperty
    def current_limit(self):
        """Integer to limit motor currents to avoid damage, see knowledge
        base for specific hand"""
        pass

    @abstractproperty
    def sensitive_motors(self):
        """List of motors (with tendons) that should not surpass current
        limit"""
        pass

    @abstractproperty
    def current_ports(self):
        """Dict that specifies which mainboard property holds the current
        readings of the hand motors as in {"motor_name": "board_attribute"}"""
        pass

    @abstractproperty
    def poses(self):
        """Nested dict that specifies position and speed of involved motors
        for each pose as in {"poseName": {"motor_name": (pos, speed_fract)}}"""
        pass

    def __init__(self, robot, isLeft, monitorCurrents=True, vrep=False):
        self.logger = logging.getLogger(__name__)

        if isLeft:
            self.prefix = "l_"
        else:
            self.prefix = "r_"

        # get hand motor accessors from robot
        if not vrep:
            self.board = getattr(robot, self.prefix + "virtualhand_x")
        for motor in self.current_ports.keys():
            setattr(self, motor, getattr(robot, self.prefix + motor))

        # genereate named methods for poses
        def add_pose_method(pose):

            def pose_func(self, fraction_max_speed, percentage):
                self.executePose(pose, fraction_max_speed, percentage)
            setattr(AbstractHand, pose, pose_func)

            pose_func.__name__ = pose
            pose_func.__doc__ = "Executes the {} pose".format(pose)

        for pose_name in self.poses.keys():
            add_pose_method(pose_name)

        self.mutex = Semaphore()

        self.motor_directions = dict(
            zip(self.sensitive_motors, ["idle"] * len(self.sensitive_motors)))

        if monitorCurrents and not vrep:
            t = threading.Thread(target=self._current_check)
            t.daemon = True
            t.start()

    def setAngle(self, motor_name, position, fraction_max_speed):
        """
        Moves motor to given position.

        :param motor_name: motor name
        :type motor_name: str
        :param position: goal position/angle
        :type position: float
        :param fraction_max_speed: Percentage of goal speed at which the motor
                                 should operate [0.0, 1.0]
        :type fraction_max_speed: float
        """
        if self.isHandMotor(motor_name):
            motor_name = motor_name[2:]
            motor = getattr(self, motor_name)

            self.mutex.acquire()
            if motor_name in self.sensitive_motors:
                if position > motor.present_position:
                    self.motor_directions[motor_name] = "closing"
                else:
                    self.motor_directions[motor_name] = "opening"

            motor.compliant = False
            motor.goal_speed = 1000.0 * fraction_max_speed
            motor.goal_position = position
            self.mutex.release()
        else:
            self.logger.warning(
                "Trying to move unknown motor {}".format(motor_name))

    def isHandMotor(self, jointname):
        """
        Checks whether the given motor belongs to the hand

        :param jointname: Name of the motor
        :type jointname: str
        :return: True if motor is a hand motor, False else
        :rtype: boolean
        """
        if jointname.startswith(self.prefix) and hasattr(self, jointname[2:]):
            return True
        return False

    def getPresentCurrent(self, jointname):
        """
        Returns the current reading for the given joint from the hand's
        mainboard. (Current readings are not stored in the motors themselves)

        :param jointName: Name of the joint
        :type jointName: str
        :return: Current of the joint
        :rtype: float
        """

        if self.isHandMotor(jointname):
            return self.board.present_motor_currents[
                self.current_ports[jointname[2:]]]

        self.logger.warning("{} is not a joint of {}Hand".format(
            jointname, self.prefix[0].upper()))
        return 0

    def executePose(self, poseName, fractionMaxSpeed=1., percentage=1.):
        """
        Executes given pose.

        :param fractionMaxSpeed: Speed at which to execute the pose.
                                 Default: 1.0
        :type fractionMaxSpeed: float
        :param percentage: Percentage of the pose to execute.
                           0.0 < percentage <= 1.0 (default)
        :type percentage: float
        """
        if poseName not in self.poses.keys():
            self.logger.warning((
                "Unknown pose {} - known poses are {}"
            ).format(poseName, self.poses.keys()))
            return

        pose = self.poses[poseName]
        for motor in pose.keys():
            angle, speed = pose[motor]
            if hasattr(self, motor):
                self.setAngle(self.prefix + motor, angle * percentage,
                              speed * fractionMaxSpeed)
            else:
                self.logger.warning(
                    "Unknown motor {} in {}".format(motor, poseName))

    def _current_check(self):
        """Thread to halt movement of sensitive motors when they exceed the
        current limit"""
        while True:
            before = time.time()
            self.mutex.acquire()
            for motor_name in self.sensitive_motors:
                if self.motor_directions[motor_name] == "closing":
                    if (self.board.present_motor_currents[
                            self.current_ports[motor_name]] >
                            self.current_limit):

                        self.logger.warning(
                            (
                                "Reached maximum current - Stopping " +
                                "movement of {}{}"
                            ).format(self.prefix, motor_name))

                        motor = getattr(self, motor_name)

                        motor.goal_position = motor.present_position
                        self.motor_directions[motor_name] = "idle"
            self.mutex.release()
            time.sleep(max(0, .1 - (time.time() - before)))
