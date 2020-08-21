#!/usr/bin/env python

import argparse
import cv_bridge
import logging
import rospy
import sys
import nicomsg.msg
import nicomsg.srv
import sensor_msgs.msg

from Motion import NicoRosMotion

from nicomotion.Motion import Motion
from nicovision.PyrepDevice import PyrepDevice
from os.path import dirname, abspath, realpath
from std_srvs.srv import Empty


class NicoRosPyrep(NicoRosMotion):
    """
    The NicoRosPyrep class provides access to pyrep based Motion module and
    streams pictures of vision sensors at each simulation steps.
    To run with default parameters use
    "rosrun nicoros Pyrep.py" (add -h to list optional parameters)
    """

    @staticmethod
    def getConfig():
        """
        Returns a default config dict

        :return: dict
        """
        return {
            "motion_config": realpath(
                dirname(abspath(__file__)) + "/../../../../json/nico_humanoid_vrep.json"
            ),
            "vrep_scene": "",
            "headless": False,
            "mode": "stereo",
            "device_left": "",
            "device_right": "",
            "width": 640,
            "height": 480,
            "near_clipping_plane": 1e-2,
            "far_clipping_plane": 10.0,
            "view_angle": 60.0,
            "rostopic_prefix": "/nico/pyrep",
            "motion_prefix": "/motion",
            "vision_prefix": "/vision",
            "rostopic_left": "/left",
            "rostopic_right": "/right",
        }

    def __init__(self, config=None):
        """
        The NicoRosPyrep enables controll of the simulated robot and sending of a
        vision sensor images through ROS.
        To run with default parameters use "rosrun nicoros Pyrep.py"
        (add -h to list optional parameters)

        :param config: Configuration dict
        :type config: dict
        """
        self._logger = logging.getLogger(__name__)
        self._config = config
        if config is None:
            self._config = NicoRosPyrep.getConfig()
        # -- vision setup --
        self._bridge = cv_bridge.CvBridge()
        self._device_left = None
        self._device_right = None
        self._stream_running = False
        # -- motion --
        vrepConfig = Motion.pyrepConfig()
        vrepConfig["vrep_scene"] = config["vrep_scene"]
        vrepConfig["headless"] = config["headless"]
        self.robot = Motion(
            motorConfig=config["motion_config"], vrep=True, vrepConfig=vrepConfig,
        )
        self.fakeJointStates = {}  # needed for inherited callback
        self._logger.info("-- Init NicoRosPyrep --")
        rospy.init_node("NicoRosPyrep", anonymous=True)
        self._logger.debug("Init ROS publishers")
        # -- pyrep vision --
        if self._config["mode"] in ("stereo", "left"):
            self._publisher_left = rospy.Publisher(
                config["rostopic_prefix"]
                + config["vision_prefix"]
                + config["rostopic_left"],
                sensor_msgs.msg.Image,
                queue_size=1,
            )
        if self._config["mode"] in ("stereo", "right"):
            self._publisher_right = rospy.Publisher(
                config["rostopic_prefix"]
                + config["vision_prefix"]
                + config["rostopic_right"],
                sensor_msgs.msg.Image,
                queue_size=1,
            )
        # -- pyrep motion --
        motion_prefix = config["rostopic_prefix"] + config["motion_prefix"]
        rospy.Subscriber(
            "%s/openHand" % motion_prefix, nicomsg.msg.s, self._ROSPY_openHand,
        )
        rospy.Subscriber(
            "%s/closeHand" % motion_prefix, nicomsg.msg.s, self._ROSPY_closeHand,
        )
        rospy.Subscriber(
            "%s/setAngle" % motion_prefix, nicomsg.msg.sff, self._ROSPY_setAngle,
        )
        rospy.Subscriber(
            "%s/changeAngle" % motion_prefix, nicomsg.msg.sff, self._ROSPY_changeAngle,
        )
        rospy.Subscriber(
            "%s/setMaximumSpeed" % motion_prefix,
            nicomsg.msg.f,
            self._ROSPY_setMaximumSpeed,
        )
        rospy.Subscriber(
            "%s/setStiffness" % motion_prefix, nicomsg.msg.sf, self._ROSPY_setStiffness,
        )
        rospy.Subscriber(
            "%s/setPID" % motion_prefix, nicomsg.msg.sfff, self._ROSPY_setPID,
        )
        rospy.Subscriber(
            "%s/enableTorque" % motion_prefix, nicomsg.msg.s, self._ROSPY__enableTorque,
        )
        rospy.Subscriber(
            "%s/disableTorque" % motion_prefix,
            nicomsg.msg.s,
            self._ROSPY__disableTorque,
        )
        rospy.Subscriber(
            "%s/enableTorqueAll" % motion_prefix,
            nicomsg.msg.empty,
            self._ROSPY__enableTorqueAll,
        )
        rospy.Subscriber(
            "%s/disableTorqueAll" % motion_prefix,
            nicomsg.msg.empty,
            self._ROSPY__disableTorqueAll,
        )
        rospy.Subscriber(
            "%s/toSafePosition" % motion_prefix,
            nicomsg.msg.empty,
            self._ROSPY__toSafePosition,
        )

        self._logger.debug("Init ROS services")
        # -- vision --

        # placeholder

        # -- motion --
        rospy.Service(
            "%s/getConfig" % motion_prefix,
            nicomsg.srv.GetString,
            self._ROSPY_getConfig,
        )
        rospy.Service(
            "%s/getVrep" % motion_prefix, nicomsg.srv.GetString, self._ROSPY_getVrep,
        )
        rospy.Service(
            "%s/getAngle" % motion_prefix, nicomsg.srv.GetValue, self._ROSPY_getAngle,
        )
        rospy.Service(
            "%s/getJointNames" % motion_prefix,
            nicomsg.srv.GetNames,
            self._ROSPY_getJointNames,
        )
        rospy.Service(
            "%s/getAngleUpperLimit" % motion_prefix,
            nicomsg.srv.GetValue,
            self._ROSPY_getAngleUpperLimit,
        )
        rospy.Service(
            "%s/getAngleLowerLimit" % motion_prefix,
            nicomsg.srv.GetValue,
            self._ROSPY_getAngleLowerLimit,
        )
        rospy.Service(
            "%s/getTorqueLimit" % motion_prefix,
            nicomsg.srv.GetValue,
            self._ROSPY_getTorqueLimit,
        )
        rospy.Service(
            "%s/getTemperature" % motion_prefix,
            nicomsg.srv.GetValue,
            self._ROSPY_getTemperature,
        )
        rospy.Service(
            "%s/getCurrent" % motion_prefix,
            nicomsg.srv.GetValue,
            self._ROSPY_getCurrent,
        )
        rospy.Service(
            "%s/getStiffness" % motion_prefix,
            nicomsg.srv.GetValue,
            self._ROSPY_getStiffness,
        )
        rospy.Service(
            "%s/getPID" % motion_prefix, nicomsg.srv.GetPID, self._ROSPY_getPID,
        )
        # Pyrep
        rospy.Service(
            "%s/getPose" % config["rostopic_prefix"],
            nicomsg.srv.GetValues,
            self._ROSPY_getPose,
        )
        rospy.Service(
            "%s/nextSimulationStep" % config["rostopic_prefix"],
            Empty,
            self._ROSPY__nextSimulationStep,
        )
        rospy.Service(
            "%s/startSimulation" % config["rostopic_prefix"],
            Empty,
            self._ROSPY__startSimulation,
        )
        rospy.Service(
            "%s/stopSimulation" % config["rostopic_prefix"],
            Empty,
            self._ROSPY__stopSimulation,
        )
        self._logger.info("-- All done --")

    def start_stream(self):
        """
        Starts the stream
        """
        if self._stream_running:
            self._logger.warning("Stream already running")
            return

        nico_eyes = PyrepDevice.autodetect_nicoeyes()
        if self._config["mode"] in ("stereo", "left"):
            if self._config["device_left"] == "":
                device = nico_eyes[0]
            else:
                device = self._config["device_left"]
            self._device_left = PyrepDevice(
                device,
                self._config["width"],
                self._config["height"],
                self._config["near_clipping_plane"],
                self._config["far_clipping_plane"],
                self._config["view_angle"],
            )
            if self._device_left is None:
                self._logger.error(
                    "Can not initialise left device - is the device name "
                    + "correct and not ambiguous?"
                )
            callback = self.create_callback(self._publisher_left)
            self._device_left.add_callback(callback)

        if self._config["mode"] in ("stereo", "right"):
            if self._config["device_right"] == "":
                device = nico_eyes[1]
            else:
                device = self._config["device_right"]
            self._device_right = PyrepDevice(
                device,
                self._config["width"],
                self._config["height"],
                self._config["near_clipping_plane"],
                self._config["far_clipping_plane"],
                self._config["view_angle"],
            )
            if self._device_right is None:
                self._logger.error(
                    "Can not initialise right device - is the device name "
                    + "correct and not ambiguous?"
                )
            callback = self.create_callback(self._publisher_right)
            self._device_right.add_callback(callback)

        self._stream_running = True

    def stop_stream(self):
        """
        Stops the stream
        """
        if not self._stream_running:
            self._logger.warning("Stream not running")
            return
        self._device_left.clean_callbacks()
        self._device_right.clean_callbacks()
        self._stream_running = False

    def is_running(self):
        """
        Returns true if stream is currently running

        :return: true if running
        :rtype: bool
        """
        return self._stream_running

    def create_callback(self, publisher):
        """
        Creates a callback to publish each frame.

        :param publisher: videowriter to write the frames
        :type publisher: rospy.publisher
        :return: the callback function
        :rtype: function
        """

        def callback(frame):
            publisher.publish(self._bridge.cv2_to_imgmsg(frame, "bgr8"))

        return callback


if __name__ == "__main__":
    # TODO arguments
    config = NicoRosPyrep.getConfig()

    parser = argparse.ArgumentParser(description="NICO ROS pyrep interface")
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
        help="Path to log file. Default: NICO_PYREP.log",
        type=str,
        default="NICO_PYREP.log",
    )
    parser.add_argument(
        "-m",
        "--motor-file",
        dest="motion_config",
        help=("Path to robot motor file. Default: " + config["motion_config"]),
        type=str,
    )
    parser.add_argument(
        "--vrep-scene",
        dest="vrep_scene",
        help=("Scene to load in VREP. Default: " + config["vrep_scene"]),
        type=str,
    )
    parser.add_argument(
        "--motion-prefix",
        dest="motion_prefix",
        help=(
            "Prefix for motion related ROS topics. Default:" + config["motion_prefix"]
        ),
        type=str,
    )
    parser.add_argument(
        "--headless",
        dest="headless",
        help=("Run vrep in headless mode"),
        action="store_true",
    )
    parser.add_argument(
        "-md",
        "--mode",
        dest="mode",
        help="Streaming mode. (left, right, stereo) "
        + "Default: {}".format(config["mode"]),
        type=str,
    )
    parser.add_argument(
        "-l",
        "--sensor-left",
        dest="device_left",
        help="Left vision sensor. Autodetected if not set",
        type=str,
    )
    parser.add_argument(
        "-r",
        "--sensor-right",
        dest="device_right",
        help="Right vision sensor. Autodetected if not set",
        type=str,
    )
    parser.add_argument(
        "-W",
        "--width",
        dest="width",
        help="Image width. Default: %i" % config["width"],
        type=float,
    )
    parser.add_argument(
        "-H",
        "--height",
        dest="height",
        help="Image height. Default: %i" % config["height"],
        type=float,
    )
    parser.add_argument(
        "--near_clipping_plane",
        dest="near_clipping_plane",
        help="Minimum detection range of the vision sensor. Default: 1e-2",
        type=float,
    )
    parser.add_argument(
        "--far_clipping_plane",
        dest="far_clipping_plane",
        help="Maximum detection range of the vision sensor. Default: 10.0",
        type=float,
    )
    parser.add_argument(
        "--view_angle",
        dest="view_angle",
        help="Degrees of field of view of the vision sensor. Default: 60.0",
        type=float,
    )
    parser.add_argument(
        "--rostopic-prefix",
        dest="rostopic_prefix",
        help="Prefix for all ROS topics. Default: %s" % config["rostopic_prefix"],
        type=str,
    )
    parser.add_argument(
        "--vision-prefix",
        dest="vision_prefix",
        help="Prefix for vision related ROS topics. Default: %s"
        % config["vision_prefix"],
        type=str,
    )
    parser.add_argument(
        "--rostopic-left",
        dest="rostopic_left",
        help="ROS topic name for left sensor. Default: %s" % config["rostopic_left"],
        type=str,
    )
    parser.add_argument(
        "--rostopic-right",
        dest="rostopic_right",
        help="ROS topic name for right sensor. Default: %s" % config["rostopic_right"],
        type=str,
    )

    args = parser.parse_known_args()[0]

    # Parse args
    if args.motion_config:
        config["motion_config"] = args.motion_config
    if args.vrep_scene:
        config["vrep_scene"] = args.vrep_scene
    if args.motion_prefix:
        config["motion_prefix"] = args.motion_prefix

    config["headless"] = args.headless

    if args.mode:
        config["mode"] = args.mode
    if args.device_left:
        config["device_left"] = args.device_left
    if args.device_right:
        config["device_right"] = args.device_right
    if args.width:
        config["width"] = args.width
    if args.height:
        config["height"] = args.height
    if args.near_clipping_plane:
        config["near_clipping_plane"] = args.near_clipping_plane
    if args.far_clipping_plane:
        config["far_clipping_plane"] = args.far_clipping_plane
    if args.view_angle:
        config["view_angle"] = args.view_angle
    if args.rostopic_prefix:
        config["rostopic_prefix"] = args.rostopic_prefix
    if args.vision_prefix:
        config["vision_prefix"] = args.vision_prefix
    if args.rostopic_left:
        config["rostopic_left"] = args.rostopic_left
    if args.rostopic_right:
        config["rostopic_right"] = args.rostopic_right

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
        filename=args.logFile,
        format="%(asctime)s %(levelname)s at %(funcName)s "
        + "(%(module)s: %(lineno)d): %(message)s",
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

    vision = NicoRosPyrep(config)
    vision.start_stream()
    rospy.spin()
    vision.stop_stream()
