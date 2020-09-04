#!/usr/bin/env python

import argparse
import logging
import sys
import threading
import time

import nicomsg.msg
import nicomsg.srv
import rospy
import sensor_msgs.msg
from nicomotion.Motion import Motion
from std_srvs.srv import Empty

try:
    from nicomoveit import moveitWrapper
except ImportError:
    pass


class NicoRosMotion:
    """
    The NicoRosMotion class exposes the functions of :class:`nicomotion.Motion`
    to ROS and
    periodically publishes the current joint states
    """

    @staticmethod
    def getConfig():
        """
        Returns a default config dict

        :return: dict
        """
        return {
            "logFile": "NICO.log",
            "robotMotorFile": "config.json",
            "vrep": False,
            "vrepHost": "127.0.0.1",
            "vrepPort": 19997,
            "vrepScene": "",
            "rostopicName": "/nico/motion",
            "jointStateName": "/joint_states",
            "fakeExecution": False,
            "usePyrep": False,
            "headless": False,
        }

    def __init__(self, config=None):
        """
        RosNicoMotion provides :class:`nicomotion.Motion` functions over ROS
        and periodically publishes the current joint states

        :param config: Configuration of the :class:`nicomotion.Motion` and
                       RosNicoMotion interface
        :type config: dict
        """
        self.logger = logging.getLogger(__name__)
        self.robot = None
        if config is None:
            config = NicoRosMotion.getConfig()

        if rospy.has_param(config["rostopicName"] + "/robotMotorFile"):
            config["robotMotorFile"] = rospy.get_param(
                config["rostopicName"] + "/robotMotorFile"
            )
        if rospy.has_param(config["rostopicName"] + "/vrep"):
            config["vrep"] = rospy.get_param(config["rostopicName"] + "/vrep")
        if rospy.has_param(config["rostopicName"] + "/vrepScene"):
            config["vrepScene"] = rospy.get_param(config["rostopicName"] + "/vrepScene")
        if rospy.has_param(config["rostopicName"] + "/fakeExecution"):
            config["fakeExecution"] = rospy.get_param(
                config["rostopicName"] + "/fakeExecution"
            )
        if rospy.has_param(config["rostopicName"] + "/pyrep"):
            config["pyrep"] = rospy.get_param(config["rostopicName"] + "/pyrep")
        if rospy.has_param(config["rostopicName"] + "/headless"):
            config["headless"] = rospy.get_param(config["rostopicName"] + "/headless")

        # init Motion
        self.logger.info("-- Init NicoRosMotion --")
        if config["pyrep"]:
            vrepConfig = Motion.pyrepConfig()
            vrepConfig["vrep_scene"] = config["vrepScene"]
            vrepConfig["headless"] = config["headless"]
        else:
            vrepConfig = Motion.vrepRemoteConfig()
            vrepConfig["vrep_scene"] = config["vrepScene"]
            vrepConfig["vrep_host"] = config["vrepHost"]
            vrepConfig["vrep_port"] = config["vrepPort"]
        self.robot = Motion(
            motorConfig=config["robotMotorFile"],
            vrep=config["vrep"],
            vrepConfig=vrepConfig,
        )

        # init ROS
        self.logger.debug("Init ROS")
        rospy.init_node("nicorosmotion", anonymous=True)

        # setup subscriber
        self.logger.debug("Init subscriber")
        rospy.Subscriber(
            "%s/openHand" % config["rostopicName"], nicomsg.msg.s, self._ROSPY_openHand
        )
        rospy.Subscriber(
            "%s/closeHand" % config["rostopicName"],
            nicomsg.msg.s,
            self._ROSPY_closeHand,
        )
        rospy.Subscriber(
            "%s/enableForceControlAll" % config["rostopicName"],
            nicomsg.msg.i,
            self._ROSPY_enableForceControlAll,
        )
        rospy.Subscriber(
            "%s/disableForceControlAll" % config["rostopicName"],
            nicomsg.msg.empty,
            self._ROSPY_disableForceControlAll,
        )
        rospy.Subscriber(
            "%s/enableForceControl" % config["rostopicName"],
            nicomsg.msg.si,
            self._ROSPY_enableForceControl,
        )
        rospy.Subscriber(
            "%s/disableForceControl" % config["rostopicName"],
            nicomsg.msg.s,
            self._ROSPY_disableForceControl,
        )
        rospy.Subscriber(
            "%s/setAngle" % config["rostopicName"],
            nicomsg.msg.sff,
            self._ROSPY_setAngle,
        )
        rospy.Subscriber(
            "%s/changeAngle" % config["rostopicName"],
            nicomsg.msg.sff,
            self._ROSPY_changeAngle,
        )
        rospy.Subscriber(
            "%s/setMaximumSpeed" % config["rostopicName"],
            nicomsg.msg.f,
            self._ROSPY_setMaximumSpeed,
        )
        rospy.Subscriber(
            "%s/setStiffness" % config["rostopicName"],
            nicomsg.msg.sf,
            self._ROSPY_setStiffness,
        )
        rospy.Subscriber(
            "%s/setPID" % config["rostopicName"], nicomsg.msg.sfff, self._ROSPY_setPID
        )
        rospy.Subscriber(
            "%s/enableTorque" % config["rostopicName"],
            nicomsg.msg.s,
            self._ROSPY__enableTorque,
        )
        rospy.Subscriber(
            "%s/disableTorque" % config["rostopicName"],
            nicomsg.msg.s,
            self._ROSPY__disableTorque,
        )
        rospy.Subscriber(
            "%s/enableTorqueAll" % config["rostopicName"],
            nicomsg.msg.empty,
            self._ROSPY__enableTorqueAll,
        )
        rospy.Subscriber(
            "%s/disableTorqueAll" % config["rostopicName"],
            nicomsg.msg.empty,
            self._ROSPY__disableTorqueAll,
        )
        rospy.Subscriber(
            "%s/toSafePosition" % config["rostopicName"],
            nicomsg.msg.empty,
            self._ROSPY__toSafePosition,
        )

        # setup services
        self.logger.debug("Init services")
        rospy.Service(
            "%s/getConfig" % config["rostopicName"],
            nicomsg.srv.GetString,
            self._ROSPY_getConfig,
        )
        rospy.Service(
            "%s/getVrep" % config["rostopicName"],
            nicomsg.srv.GetString,
            self._ROSPY_getVrep,
        )
        rospy.Service(
            "%s/getAngle" % config["rostopicName"],
            nicomsg.srv.GetValue,
            self._ROSPY_getAngle,
        )
        rospy.Service(
            "%s/getPose" % config["rostopicName"],
            nicomsg.srv.GetValues,
            self._ROSPY_getPose,
        )
        rospy.Service(
            "%s/getJointNames" % config["rostopicName"],
            nicomsg.srv.GetNames,
            self._ROSPY_getJointNames,
        )
        rospy.Service(
            "%s/getAngleUpperLimit" % config["rostopicName"],
            nicomsg.srv.GetValue,
            self._ROSPY_getAngleUpperLimit,
        )
        rospy.Service(
            "%s/getAngleLowerLimit" % config["rostopicName"],
            nicomsg.srv.GetValue,
            self._ROSPY_getAngleLowerLimit,
        )
        rospy.Service(
            "%s/getTorqueLimit" % config["rostopicName"],
            nicomsg.srv.GetValue,
            self._ROSPY_getTorqueLimit,
        )
        rospy.Service(
            "%s/getTemperature" % config["rostopicName"],
            nicomsg.srv.GetValue,
            self._ROSPY_getTemperature,
        )
        rospy.Service(
            "%s/getCurrent" % config["rostopicName"],
            nicomsg.srv.GetValue,
            self._ROSPY_getCurrent,
        )
        rospy.Service(
            "%s/getStiffness" % config["rostopicName"],
            nicomsg.srv.GetValue,
            self._ROSPY_getStiffness,
        )
        rospy.Service(
            "%s/getPID" % config["rostopicName"], nicomsg.srv.GetPID, self._ROSPY_getPID
        )

        rospy.Service(
            "%s/nextSimulationStep" % config["rostopicName"],
            Empty,
            self._ROSPY__nextSimulationStep,
        )
        rospy.Service(
            "%s/startSimulation" % config["rostopicName"],
            Empty,
            self._ROSPY__startSimulation,
        )
        rospy.Service(
            "%s/stopSimulation" % config["rostopicName"],
            Empty,
            self._ROSPY__stopSimulation,
        )

        # setup class variables
        self._running = True
        self.jsonConfig = self.robot.getConfig()
        self.vrep = self.robot.getVrep()
        self.config = config
        self.fakeJointStates = {}

        # setup publishers
        self.logger.debug("Init publishers")
        self._palm_publisher_left = rospy.Publisher(
            "%s/palm_sensor/left" % config["rostopicName"],
            nicomsg.msg.i,
            queue_size=10,
        )
        self._palm_publisher_right = rospy.Publisher(
            "%s/palm_sensor/right" % config["rostopicName"],
            nicomsg.msg.i,
            queue_size=10,
        )
        self._palm_thread = threading.Thread(target=self._palm_sensor_publisher)
        self._palm_thread.start()

        if "nicomoveit.moveitWrapper" in sys.modules:
            self._jointStatePublisher = rospy.Publisher(
                "%s" % config["jointStateName"],
                sensor_msgs.msg.JointState,
                queue_size=1,
            )
            # a asynchronous thread is seperated from the main thread
            self._jointStateThread = threading.Thread(target=self._sendJointState)
            self._jointStateThread.start()
        else:
            self._jointStateThread = None

        # wait for messages
        self.logger.info("-- All done --")

    def stop(self):
        self._running = False
        if self._jointStateThread:
            self._jointStateThread.join()
        self._palm_thread.join()

    def _ROSPY_openHand(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.openHand`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.robot.openHand(message.param1)

    def _ROSPY_closeHand(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.closeHand`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.robot.closeHand(message.param1)

    def _ROSPY_enableForceControlAll(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.enableForceControlAll`

        :param message: ROS message
        :type message: nicomsg.msg.i
        """
        self.robot.enableForceControlAll(message.param1)

    def _ROSPY_disableForceControlAll(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.disableForceControlAll`

        :param message: ROS message
        :type message: nicomsg.msg.empty
        """
        self.robot.disableForceControlAll()

    def _ROSPY_enableForceControl(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.enableForceControl`

        :param message: ROS message
        :type message: nicomsg.msg.si
        """
        self.robot.enableForceControl(message.param1, message.param2)

    def _ROSPY_disableForceControl(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.disableForceControl`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.robot.disableForceControl(message.param1)

    def _ROSPY_setAngle(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.setAngles`

        :param message: ROS message
        :type message: nicomsg.msg.sff
        """
        self.fakeJointStates[message.param1] = message.param2
        self.robot.setAngle(message.param1, message.param2, message.param3)

    def _ROSPY_changeAngle(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.changeAngles`

        :param message: ROS message
        :type message: nicomsg.msg.sff
        """
        self.robot.changeAngle(message.param1, message.param2, message.param3)

    def _ROSPY_getVrep(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getVrep`

        :param message: ROS message
        :type message: nicomsg.srv.GetString
        :return: 'True' if vrep is used, 'False' if real NICO is used
        :rtype: string
        """
        return str(self.robot.getVrep())

    def _ROSPY_getConfig(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getConfig`

        :param message: ROS message
        :type message: nicomsg.srv.GetString
        :return: Dictionary with motor config that has been converted to a
                 string
        :rtype: string
        """
        return str(self.robot.getConfig())

    def _ROSPY_getAngle(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getAngle`

        :param message: ROS message
        :type message: nicomsg.srv.GetValue
        :return: Angle of requested joint
        :rtype: float
        """
        return self.robot.getAngle(message.param1)

    def _ROSPY_getPose(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getPose`

        :param message: ROS message
        :type message: nicomsg.srv.GetValues
        :return: Position of the requestet object in x,y,z coordinates
        :rtype: list
        """
        return [self.robot.getPose(message.param1)]

    def _ROSPY_getJointNames(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getJointNames`

        :param message: ROS message
        :type message: nicomsg.srv.GetNames
        :return: List of joint names
        :rtype: list
        """
        return [self.robot.getJointNames()]

    def _ROSPY_getAngleUpperLimit(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getAngleUpperLimit`

        :param message: ROS message
        :type message: nicomsg.srv.GetValue
        :return: Angle upper limit of requested joint
        :rtype: float
        """
        return self.robot.getAngleUpperLimit(message.param1)

    def _ROSPY_getAngleLowerLimit(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getAngleLowerLimit`

        :param message: ROS message
        :type message: nicomsg.srv.GetValue
        :return: Angle lower limit of requested joint
        :rtype: float
        """
        return self.robot.getAngle(message.param1)

    def _ROSPY_getTorqueLimit(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getTorqueLimit`

        :param message: ROS message
        :type message: nicomsg.srv.GetValue
        :return: Torque limit of requested joint
        :rtype: float
        """
        return self.robot.getTorqueLimit(message.param1)

    def _ROSPY_getTemperature(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getTemperature`

        :param message: ROS message
        :type message: nicomsg.srv.GetValue
        :return: Temperature of requested joint
        :rtype: float
        """
        return self.robot.getTemperature(message.param1)

    def _ROSPY_getCurrent(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getCurrent`

        :param message: ROS message
        :type message: nicomsg.srv.GetValue
        :return: Current of requested joint
        :rtype: float
        """
        return self.robot.getCurrent(message.param1)

    def _ROSPY_setMaximumSpeed(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.setMaximumSpeed`

        :param message: ROS message
        :type message: nicomsg.msg.f
        """
        self.robot.setMaximumSpeed(message.param1)

    def _ROSPY_setStiffness(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.setStiffness`

        :param message: ROS message
        :type message: nicomsg.msg.sf
        """
        self.robot.setStiffness(message.param1, message.param2)

    def _ROSPY_getStiffness(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getStiffness`

        :param message: ROS message
        :type message: nicomsg.srv.GetValue
        :return: Stiffness of requested joint
        :rtype: float
        """
        return self.robot.getStiffness(message.param1)

    def _ROSPY_setPID(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.setPID`

        :param message: ROS message
        :type message: nicomsg.msg.sfff
        """
        self.robot.setPID(
            message.param1, message.param2, message.param3, message.param4
        )

    def _ROSPY_getPID(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.getPID`

        :param message: ROS message
        :type message: nicomsg.srv.GetPID
        :return: Tuple: (p, i, d)
        :rtype: tuple
        """
        return self.robot.getPID(message.param1)

    def _ROSPY__enableTorque(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.enableTorque`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.robot.enableTorque(message.param1)

    def _ROSPY__disableTorque(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.disableTorque`

        :param message: ROS message
        :type message: nicomsg.msg.s
        """
        self.robot.disableTorque(message.param1)

    def _ROSPY__enableTorqueAll(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.enableTorqueAll`

        :param message: ROS message
        :type message: nicomsg.msg.empty
        """
        self.robot.enableTorqueAll()

    def _ROSPY__disableTorqueAll(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.disableTorqueAll`

        :param message: ROS message
        :type message: nicomsg.msg.empty
        """
        self.robot.disableTorqueAll()

    def _ROSPY__toSafePosition(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.toSafePosition`

        :param message: ROS message
        :type message: nicomsg.msg.empty
        """
        self.robot.toSafePosition()

    def _ROSPY__nextSimulationStep(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.nextSimulationStep`

        :param message: ROS message
        :type message: std_srvs.srv.Empty
        """
        self.robot.nextSimulationStep()
        return []

    def _ROSPY__startSimulation(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.startSimulation`

        :param message: ROS message
        :type message: std_srvs.srv.Empty
        """
        self.robot.startSimulation()
        return []

    def _ROSPY__stopSimulation(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.stopSimulation`

        :param message: ROS message
        :type message: std_srvs.srv.Empty
        """
        self.robot.stopSimulation()
        return []

    def _sendJointState(self):
        """
        Loop for sending the current joint state
        """
        while self._running:
            if rospy.has_param(self.config["rostopicName"] + "/fakeExecution"):
                self.config["fakeExecution"] = rospy.get_param(
                    self.config["rostopicName"] + "/fakeExecution"
                )
            message = sensor_msgs.msg.JointState()
            message.name = []
            message.position = []
            message.effort = []
            joints = self.robot.getJointNames()

            for joint in joints:
                if self.config["fakeExecution"] and joint in self.fakeJointStates:
                    value = self.fakeJointStates[joint]
                else:
                    value = self.robot.getAngle(joint)
                message.name += [joint]
                value = moveitWrapper.nicoToRosAngle(
                    joint, value, self.jsonConfig, self.vrep
                )
                rospy.loginfo(joint + " " + str(value))
                message.position += [value]
                # message.effort += [self.robot.getLoad(joint)]

            # to avoid a warning about missing joint values a joint value of
            # 0.0 is published for the joints with unknown joint value
            joints_without_motor = [
                "l_indexfinger_1st_x",
                "l_indexfinger_2nd_x",
                "l_ringfingers_x",
                "l_ringfinger_1st_x",
                "l_ringfinger_2nd_x",
                "l_thumb_1st_x",
                "l_thumb_2nd_x",
                "r_indexfinger_1st_x",
                "r_indexfinger_2nd_x",
                "r_ringfingers_x",
                "r_ringfinger_1st_x",
                "r_ringfinger_2nd_x",
                "r_thumb_1st_x",
                "r_thumb_2nd_x",
            ]
            for joint in joints_without_motor:
                message.name += [joint]
                value = 0.0
                message.position += [value]

            message.header.stamp = rospy.get_rostime()
            self._jointStatePublisher.publish(message)
            time.sleep(0.25)

    def __del__(self):
        self.robot.cleanup()

    def _palm_sensor_publisher(self):
        r = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            left = self.robot.getPalmSensorReading("l")
            right = self.robot.getPalmSensorReading("r")
            self._palm_publisher_left.publish(left)
            self._palm_publisher_right.publish(right)
            r.sleep()


if __name__ == "__main__":
    config = NicoRosMotion.getConfig()

    # Parse command line
    parser = argparse.ArgumentParser(description="NICO ROS motion interface")
    parser.add_argument(
        "--log-level",
        dest="logLevel",
        help="Sets log level. Default: INFO",
        type=str,
        default="INFO",
    )
    parser.add_argument(
        "--log-file",
        dest="logFile",
        help=("Path to log file. Default: %s" % config["logFile"]),
        type=str,
    )
    parser.add_argument(
        "-m",
        "--motor-file",
        dest="robotMotorFile",
        help=("Path to robot motor file. Default: " + config["robotMotorFile"]),
        type=str,
    )
    parser.add_argument(
        "-v",
        "--vrep",
        dest="vrep",
        help="Connect to VREP rather than to a real robot",
        action="store_true",
    )
    parser.add_argument(
        "--vrep-host",
        dest="vrepHost",
        help="Host of VREP. Default: %s" % config["vrepHost"],
        type=str,
    )
    parser.add_argument(
        "--vrep-port",
        dest="vrepPort",
        help="Port of VREP. Default: %i" % config["vrepPort"],
        type=int,
    )
    parser.add_argument(
        "--vrep-scene",
        dest="vrepScene",
        help=("Scene to load in VREP. Default: " + config["vrepScene"]),
        type=str,
    )
    parser.add_argument(
        "--rostopic-name",
        dest="rostopicName",
        help=("Topic name for ROS. Default:" + config["rostopicName"]),
        type=str,
    )
    parser.add_argument(
        "-j",
        "--joint-state-name",
        dest="jointStateName",
        help=(
            "Name of the joint state topic. PLEASE NOTE: "
            + "A lot of other nodes assume the joint state "
            + "note to be at /joint_states, so be careful "
            + "when changing it. Default: %s"
        )
        % config["jointStateName"],
        type=str,
    )
    parser.add_argument(
        "-f",
        "--fake-execution",
        dest="fake",
        help=("Publish fake joint states instead of real " + "joint states"),
        action="store_false",
    )
    parser.add_argument(
        "-p",
        "--pyrep",
        dest="pyrep",
        help=("Use pyrep instead of vrep remote api"),
        action="store_true",
    )
    parser.add_argument(
        "--headless",
        dest="headless",
        help=("Run vrep in headless mode (requires pyrep)"),
        action="store_true",
    )

    args = parser.parse_known_args()[0]
    if args.logFile:
        config["logFile"] = args.logFile
    if args.robotMotorFile:
        config["robotMotorFile"] = args.robotMotorFile
    config["vrep"] = args.vrep
    if args.vrepHost:
        config["vrepHost"] = args.vrepHost
    if args.vrepPort:
        config["vrepPort"] = args.vrepPort
    if args.vrepScene:
        config["vrepScene"] = args.vrepScene
    if args.rostopicName:
        config["rostopicName"] = args.rostopicName
    if args.jointStateName:
        config["jointStateName"] = args.jointStateName
    config["fakeExecution"] = args.fake
    config["pyrep"] = args.pyrep
    config["headless"] = args.headless

    # Set logging setting
    loggingLevel = logging.INFO
    try:
        loggingLevel = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARNING": logging.WARNING,
            "CRITICAL": logging.CRITICAL,
            "debug": logging.DEBUG,
            "info": logging.INFO,
            "warning": logging.WARNING,
            "critical": logging.CRITICAL,
        }[args.logLevel]
    except KeyError:
        sys.stderr.write("LOGGING ERROR: Unknown log level %s\n" % args.logLevel)
        pass

    logging.basicConfig(
        filename=config["logFile"],
        format=(
            "%(asctime)s %(levelname)s at %(funcName)s "
            + "(%(module)s: %(lineno)d): %(message)s"
        ),
        level=loggingLevel,
    )
    stdoutHandler = logging.StreamHandler(sys.stdout)
    stdoutHandler.setLevel(loggingLevel)
    logging_format = logging.Formatter(
        "%(asctime)s %(levelname)s at %(funcName)s (%(module)s: "
        + "%(lineno)d): %(message)s"
    )
    stdoutHandler.setFormatter(logging_format)
    logging.getLogger().addHandler(stdoutHandler)

    rosConnection = NicoRosMotion(config)

    rospy.spin()

    rosConnection.stop()
