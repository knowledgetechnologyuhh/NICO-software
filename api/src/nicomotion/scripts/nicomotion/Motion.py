import json
import logging
import pprint
import re
import subprocess
import time

import pypot.dynamixel
import pypot.dynamixel.error as pypot_error
import pypot.robot
import pypot.vrep
from pypot.dynamixel.io.abstract_io import DxlCommunicationError, DxlError
from pypot.vrep.remoteApiBindings import vrep as remote_api

from ._nicomotion_internal.MotionError import MotionErrorHandler
from ._nicomotion_internal.RH4D_hand import RH4DHand
from ._nicomotion_internal.RH5D_hand import RH5DHand
from ._nicomotion_internal.RH7D_hand import RH7DHand


class Motion:
    """
    The Motion class provides a high level interface to various movement
    related functions of the NICO robot
    """

    @staticmethod
    def vrepRemoteConfig():
        """
        Returns a default config dict

        :return: dict
        """
        return {
            "use_pyrep": False,
            "vrep_host": "127.0.0.1",
            "vrep_port": 19997,
            "vrep_scene": None,
            "tracked_objects": [],
            "tracked_collisions": [],
            "id": None,
            "shared_vrep_io": None,
        }

    @staticmethod
    def pyrepConfig():
        """
        Returns a default config dict

        :return: dict
        """
        return {
            "use_pyrep": True,
            "vrep_scene": "",
            "start": False,
            "headless": False,
            "responsive_ui": False,
            "tracked_objects": [],
            "tracked_collisions": [],
            "id": None,
            "shared_vrep_io": None,
        }

    def __init__(
        self,
        motorConfig="config.json",
        vrep=False,
        vrepConfig=None,
        ignoreMissing=False,
        monitorHandCurrents=True,
    ):
        """
        Motion is an interface to control the movement of the NICO robot.

        :param motorConfig: motor config file (JSON format)
        :type motorConfig: str
        :param vrep: If set to true VREP will be used instead of real robot
        :type vrep: bool
        :param vrepHost: Network address of VREP
        :type vrepHost: str
        :param vrepPort: Port of VREP
        :type vrepPort: int
        :param vrepScene: Scene to load. Set to None to use current scene
        :type vrepScene: str
        :param ignoreMissing: If missing motors should be removed
        :type ignoreMissing: bool
        :param monitorHandCurrents: Whether movement of sensitiv hand motors
                                    should be halted if their recommended
                                    current threshold is surpassed (do not
                                    deactivate unless absolutely necessary)
        :type monitorHandCurrents: bool
        """
        self._robot = None
        self._maximumSpeed = 1.0
        self._vrep = vrep
        self._vrepIO = None
        self._logger = logging.getLogger(__name__)

        pypot_error.BaseErrorHandler = MotionErrorHandler

        if vrepConfig is None:
            vrepConfig = Motion.vrepRemoteConfig()
        self._pyrep = vrepConfig["use_pyrep"]

        with open(motorConfig, "r") as config_file:
            config = json.load(config_file)

        self._config = config

        if vrep:
            self._logger.info("Using VREP")
            if self._pyrep:
                try:
                    from pypot import pyrep
                except ImportError as e:
                    self._logger.warning(
                        "Failed to import pyrep from pypot. Make sure you are "
                        "using Python 3 and all environment variables are set."
                    )
                    raise e
                self._robot = pyrep.from_pyrep(
                    config,
                    vrepConfig["vrep_scene"],
                    vrepConfig["start"],
                    vrepConfig["headless"],
                    vrepConfig["responsive_ui"],
                    vrepConfig["tracked_objects"],
                    vrepConfig["tracked_collisions"],
                    vrepConfig["id"],
                    vrepConfig["shared_vrep_io"],
                )
            else:
                self._robot = pypot.vrep.from_vrep(
                    config,
                    vrepConfig["vrep_host"],
                    vrepConfig["vrep_port"],
                    vrepConfig["vrep_scene"],
                    vrepConfig["tracked_objects"],
                    vrepConfig["tracked_collisions"],
                    vrepConfig["id"],
                    vrepConfig["shared_vrep_io"],
                )
                self._vrepIO = self._robot._controllers[0].io
        else:
            self._logger.info("Using robot")
            self._adjust_port_latency()

            def remove_missing(ids):
                for id in ids:
                    id = int(id)
                    for motor in list(config["motors"].keys()):
                        if config["motors"][motor]["id"] == id:
                            self._logger.warning(
                                "Removing motor {} ({})".format(motor, id)
                            )
                            config["motors"].pop(motor)
                            for group in config["motorgroups"].keys():
                                config["motorgroups"][group] = [
                                    x
                                    for x in config["motorgroups"][group]
                                    if x != motor
                                ]
                self._logger.warning("New config created:")
                self._logger.warning(pprint.pformat(config))

            retries = 0
            success = False
            while not success:
                try:
                    self._robot = pypot.robot.from_config(config)
                    success = True
                except KeyError as e:
                    # reinitialize pypot up to 3 times
                    if retries < 3:
                        # fetch ids from error
                        ids = [int(str(e))]
                        self._logger.warning(
                            "Missing id {} - Retrying initialization".format(e)
                        )
                        if ignoreMissing:
                            remove_missing(ids)
                        retries += 1
                except DxlError as e:
                    # reinitialize pypot up to 3 times
                    if retries < 3:
                        # fetch ids from error
                        regex = re.compile(r".*\((?P<ids>.*)\).*")
                        match = regex.match(e.args[0])
                        string = match.group("ids")
                        ids = string.split(",")
                        self._logger.warning(
                            "Missing ids {} - Retrying initialization".format(string)
                        )
                        if ignoreMissing:
                            remove_missing(ids)
                        retries += 1
                except IndexError as e:
                    # reinitialize pypot up to 3 times
                    if retries < 3:
                        # fetch ids from error
                        regex = re.compile(r".*\[.*\].*\[(?P<ids>.*)\].*")
                        match = regex.match(e.args[0])
                        string = match.group("ids")
                        ids = string.split(",")
                        self._logger.warning(
                            "Missing ids {} - Retrying initialization".format(string)
                        )
                        if ignoreMissing:
                            remove_missing(ids)
                        retries += 1
                    else:
                        self._logger.error(
                            ("Initialization failed after {} retries").format(retries)
                        )
                        raise e
                except RuntimeError as e:
                    # Workaround for every other init failing
                    # FIXME Find source for RuntimeError on every other init
                    if retries == 3:
                        self._logger.error(
                            ("Initialization failed after {} retries").format(retries)
                        )
                        raise e
                    retries += 1
                    self._logger.warning(
                        "Retrying initialization after an error occured"
                    )
                    time.sleep(1)
        time.sleep(3)  # wait for syncloop to initialize values
        # initialize hands
        # left hand
        if hasattr(self._robot, "l_middlefingers_x"):
            if hasattr(self._robot, "l_wrist_x"):
                self._leftHand = RH7DHand(
                    robot=self._robot,
                    isLeft=True,
                    monitorCurrents=monitorHandCurrents,
                    vrep=vrep,
                )
            else:
                self._leftHand = RH5DHand(
                    robot=self._robot,
                    isLeft=True,
                    monitorCurrents=monitorHandCurrents,
                    vrep=vrep,
                )
        elif hasattr(self._robot, "l_thumb_x"):
            self._leftHand = RH4DHand(
                robot=self._robot,
                isLeft=True,
                monitorCurrents=monitorHandCurrents,
                vrep=vrep,
            )
        # right hand
        if hasattr(self._robot, "r_middlefingers_x"):
            if hasattr(self._robot, "r_wrist_x"):
                self._rightHand = RH7DHand(
                    robot=self._robot,
                    isLeft=False,
                    monitorCurrents=monitorHandCurrents,
                    vrep=vrep,
                )
            else:
                self._rightHand = RH5DHand(
                    robot=self._robot,
                    isLeft=False,
                    monitorCurrents=monitorHandCurrents,
                    vrep=vrep,
                )
        elif hasattr(self._robot, "r_thumb_x"):
            self._rightHand = RH4DHand(
                robot=self._robot,
                isLeft=False,
                monitorCurrents=monitorHandCurrents,
                vrep=vrep,
            )
        # remember initial situation as a safe state
        self.safeState = dict()
        for motor in self._robot.motors:
            self.safeState[motor.name] = motor.present_position

    def _adjust_port_latency(self):
        """
        Lowers the latency of the NICO port to prevent communication delays
        """
        # checks whether setserial is installed
        try:
            subprocess.check_output(["which", "setserial"])
        except subprocess.CalledProcessError as e:
            self._logger.critical(
                "could not find setserial - please make "
                + "sure that the setserial package is "
                + "installed on your system"
            )
            raise e

        # get all ports
        ports = pypot.dynamixel.get_available_ports()

        if not ports:
            raise OSError("No available ports found")

        # find a port which has dynamixel motors attached
        ids = range(48)
        motors_found = False
        for port in ports:
            try:
                self._logger.info("connecting to {}".format(port))
                dxl_io = pypot.dynamixel.DxlIO(port)
                if len(dxl_io.scan(ids)) > 0:
                    motors_found = True
                    break
            except DxlCommunicationError:
                self._logger.warning(
                    "Skipping {} due to communication error".format(port)
                )

        if not motors_found:
            raise OSError("No valid port found")

        # set latency
        self._logger.info("Setting port {} to low latency".format(port))
        subprocess.call(["setserial", port, "low_latency"])

    def getVrep(self):
        """
        Returns if vrep simulation is used or not

        :return: is vrep or real NICO used
        :rtype: boolean
        """
        return self._vrep

    def getConfig(self):
        """
        Returns the JSON configuration of motors

        :return: motor configuration
        :rtype: dict
        """
        return self._config

    def startSimulation(self, synchronize=False):
        """
        Starts the V-REP Simulation. If 'synchronize' is set True the
        simulation steps will not advance until nextSimulationStep() is called.
        (NOTE: pyrep is always in synchronous mode)

        :param synchronize: Enables control over simulation time steps
        :type synchronize: bool
        """
        if self._vrep:
            if self._pyrep:
                self._robot.start_simulation()
            else:
                if synchronize:
                    remote_api.simxSynchronous(self._vrepIO.client_id, True)
                    self._vrepIO.start_simulation()
                else:
                    remote_api.simxSynchronous(self._vrepIO.client_id, False)
                    self._vrepIO.start_simulation()

        else:
            self._logger.warning("startSimulation() has no effect on a real robot")

    def setSimulationDeltatime(self, dt):
        """
        Sets the timeframe which one simulation step represents.

        Only works while the simulation is stopped and dt is set
        to custom.


        :param dt: timeframe of one simulation step in seconds
        :type dt: float
        """
        if self._vrep:
            if self._pyrep:
                self._robot.set_simulation_timestep(dt)
            else:
                self._vrepIO.call_remote_api(
                    "simxSetFloatingParameter",
                    remote_api.sim_floatparam_simulation_time_step,
                    dt,
                )
        else:
            self._logger.warning("Robot is not simulated.")

    def getSimulationDeltatime(self):
        """
        Gets the timeframe which one simulation step represents.

        :return: timeframe of one simulation step in seconds
        :rtype: float
        """
        if self._vrep:
            if self._pyrep:
                return self._robot.get_simulation_timestep()
            else:
                return self._vrepIO.call_remote_api(
                    "simxGetFloatingParameter",
                    remote_api.sim_floatparam_simulation_time_step,
                )
        else:
            self._logger.warning("Robot is not simulated.")
            return 0.0

    def nextSimulationStep(self):
        """
        Advances the V-REP simulation by one step if synchronize was set on
        startSimulation().
        """
        if self._vrep:
            if self._pyrep:
                self._robot.simulation_step()
            else:
                remote_api.simxSynchronousTrigger(self._vrepIO.client_id)
        else:
            self._logger.warning("nextSimulationStep() has no effect on a real robot")

    def stopSimulation(self):
        """
        Stops the V-REP simulation
        """
        if self._vrep:
            if self._pyrep:
                self._robot.stop_simulation()
            else:
                self._vrepIO.stop_simulation()
        else:
            self._logger.warning("stopSimulation() has no effect on a real robot")

    def resetSimulation(self):
        """
        Restarts the V-REP simulation
        """
        if self._vrep:
            if self._pyrep:
                self._robot.reset_simulation()
            else:
                self._vrepIO.restart_simulation()
        else:
            self._logger.warning("resetSimulation() has no effect on a real robot")

    def callVREPRemoteApi(self, func_name, *args, **kwargs):
        """ Calls any remote API func in a thread_safe way.

        :param str func_name: name of the remote API func to call
        :param args: args to pass to the remote API call
        :param kwargs: args to pass to the remote API call

        :return: api response

        .. note:: You can add an extra keyword to specify if you want to use
                  the streaming or sending mode. The oneshot_wait mode is used
                  by default (see `here <http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#operationModes>`_
                  for details about possible modes).

        .. warning:: You should not pass the clientId and the operationMode as
                     arguments. They will be automatically added.
        """
        if self._vrep:
            return self._vrepIO.call_remote_api(func_name, *args, **kwargs)
        else:
            self._logger.warning("callVREPRemoteApi() has no effect on a real robot")
            return None

    def getVrepIO(self):
        """
        Gives access to pypots vrep IO (see
        https://poppy-project.github.io/pypot/pypot.vrep.html#pypot.vrep.io.VrepIO)

        :return: Pypot vrep IO
        :rtype: pypot.vrep.io
        """
        if self._vrep:
            return self._vrepIO
        else:
            self._logger.warning("A real robot has no VREP controller")
            return None

    def setHandPose(self, handName, poseName, fractionMaxSpeed=1.0, percentage=1):
        """
        Executes pose with the specified hand. Most poses only works with the
        RH5D and RH7D 4-finger hands. Make sure to open the hand beforehand.

        Known poses are: ("thumbsUp", "pointAt", "okSign",
        "pinchToIndex", "keyGrip", "pencilGrip", "closeHand", "openHand").

        handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param poseName: Name of the pose ("thumbsUp", "pointAt", "okSign",
                         "pinchToIndex", "keyGrip", "pencilGrip", "closeHand",
                         "openHand")
        :type poseName: str
        :param fractionMaxSpeed: Speed at which hand move. Default: 1.0
        :type fractionMaxSpeed: float
        :param percentage: Percentage of the pose to execute.
                           0.0 < percentage <= 1.0 (default)
        :type percentage: float
        """
        if self._vrep:
            self._logger.warning("'{}' pose is not supported for vrep".format(poseName))
        else:
            if handName.lower().startswith("l"):
                hand = self._leftHand
            elif handName.lower().startswith("r"):
                hand = self._rightHand
            else:
                self._logger.warning("Unknown hand name {}".format(handName))

            if hasattr(hand, poseName):
                speed = min(fractionMaxSpeed, self._maximumSpeed)
                hand.executePose(poseName, speed, percentage)
            else:
                self._logger.warning(
                    "'{}' pose is not supported for hand model {}".format(
                        poseName, hand.__class__.__name__
                    )
                )

    def openHand(self, handName, fractionMaxSpeed=1.0, percentage=1.0):
        """
        Opens the specified hand. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
        :type fractionMaxSpeed: float
        :param percentage: Percentage hand should open. 0.0 < percentage <= 1.0
        :type percentage: float
        """
        if handName.lower().startswith("l"):
            hand = self._leftHand
        elif handName.lower().startswith("r"):
            hand = self._rightHand
        else:
            self._logger.warning("Unknown hand name {}".format(handName))

        if self._vrep:
            hand.openHandVREP(min(fractionMaxSpeed, self._maximumSpeed), percentage)
        else:
            hand.openHand(min(fractionMaxSpeed, self._maximumSpeed), percentage)

    def closeHand(self, handName, fractionMaxSpeed=1.0, percentage=1.0):
        """
        Closes the specified hand. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should close. Default: 1.0
        :type fractionMaxSpeed: float
        :param percentage: Percentage hand should open. 0.0 < percentage <= 1.0
        :type percentage: float
        """
        if handName.lower().startswith("l"):
            hand = self._leftHand
        elif handName.lower().startswith("r"):
            hand = self._rightHand
        else:
            self._logger.warning("Unknown hand name {}".format(handName))

        if self._vrep:
            hand.closeHandVREP(min(fractionMaxSpeed, self._maximumSpeed), percentage)
        else:
            hand.closeHand(min(fractionMaxSpeed, self._maximumSpeed), percentage)

    def getPalmSensorReading(self, handName):
        """
        Returns current reading of the palm IR sensor of the specified hand.

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :return: Raw IR sensor value
        :rtype: int
        """
        if handName.lower().startswith("l"):
            hand = self._leftHand
        elif handName.lower().startswith("r"):
            hand = self._rightHand
        else:
            self._logger.warning("Unknown hand name {}".format(handName))

        return hand.getPalmSensorReading()

    def enableForceControlAll(self, goalForce=500):
        """
        Enables force control for all joints which support this feature

        :param goalForce: Goal force (0-2000)
        :type goalForce: int
        """
        for motor in self._robot.motors:
            if hasattr(motor, "force_control_enable"):
                motor.force_control_enable = True
                motor.goal_force = goalForce

    def disableForceControlAll(self):
        """
        Disables force control for all joints which support this feature
        """
        for motor in self._robot.motors:
            if hasattr(motor, "force_control_enable"):
                motor.force_control_enable = False

    def enableForceControl(self, jointName, goalForce):
        """
        Enables force control for a single joint

        :param jointName: Name of the joint
        :type jointName: str
        :param goalForce: Goal force (0-2000)
        :type goalForce: int
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, "force_control_enable"):
                motor.force_control_enable = True
                motor.goal_force = goalForce
            else:
                self._logger.warning("Joint %s has no force control" % jointName)
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return

    def disableForceControl(self, jointName):
        """
        Disables force control for a single joint

        :param jointName: Name of the joint
        :type jointName: str
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, "force_control_enable"):
                motor.force_control_enable = False
            else:
                self._logger.warning("Joint %s has no force control" % jointName)
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return

    def toSafePosition(self):
        """
        Moves the robot to its initial state of this session.
        In this state it should be safe to disable the force control.
        To receive a collision free motion trajectories use the corresponding
        moveitWrapper function instead.
        """
        for motor in self.safeState:
            self.setAngle(motor, self.safeState[motor], 0.1)

    def setAngle(self, jointName, angle, fractionMaxSpeed):
        """
        Sets the angle of a given joint to an angle (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :param angle: Angle (in degree)
        :type angle: float
        :param fractionMaxSpeed: Movement speed of joint
        :type fractionMaxSpeed: float
        """
        if hasattr(self._robot, jointName):
            handMotor = False
            if hasattr(self, "_leftHand") and self._leftHand.isHandMotor(jointName):
                hand = self._leftHand
                handMotor = True
            elif hasattr(self, "_rightHand") and self._rightHand.isHandMotor(jointName):
                hand = self._rightHand
                handMotor = True

            if handMotor:
                hand.setAngle(
                    jointName, angle, min(fractionMaxSpeed, self._maximumSpeed)
                )
            else:
                motor = getattr(self._robot, jointName)
                motor.compliant = False
                motor.goal_speed = 1000.0 * min(fractionMaxSpeed, self._maximumSpeed)
                motor.goal_position = angle
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return

    def changeAngle(self, jointName, change, fractionMaxSpeed):
        """
        Changes the angle of a given joint by an angle (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :param angle: Angle (in degree)
        :type angle: float
        :param fractionMaxSpeed: Movement speed of joint
        :type fractionMaxSpeed: float
        """
        if hasattr(self._robot, jointName):
            handMotor = False
            if hasattr(self, "_leftHand") and self._leftHand.isHandMotor(jointName):
                hand = self._leftHand
                handMotor = True
            elif hasattr(self, "_rightHand") and self._rightHand.isHandMotor(jointName):
                hand = self._rightHand
                handMotor = True

            motor = getattr(self._robot, jointName)

            if handMotor:
                hand.setAngle(
                    jointName,
                    change + hand.getAngle(jointName),
                    min(fractionMaxSpeed, self._maximumSpeed),
                )
            else:
                motor.compliant = False
                motor.goal_speed = 1000.0 * min(fractionMaxSpeed, self._maximumSpeed)
                motor.goal_position = change + motor.present_position
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return

    def getAngle(self, jointName):
        """
        Returns the current angle of a given joint (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :return: Angle of the joint (degree)
        :rtype: float
        """
        if hasattr(self._robot, jointName):
            handMotor = False
            if hasattr(self, "_leftHand") and self._leftHand.isHandMotor(jointName):
                hand = self._leftHand
                handMotor = True
            elif hasattr(self, "_rightHand") and self._rightHand.isHandMotor(jointName):
                hand = self._rightHand
                handMotor = True

            if handMotor:
                return hand.getAngle(jointName)
            else:
                motor = getattr(self._robot, jointName)
                return motor.present_position
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return 0.0

    def getJointNames(self):
        """
        Returns all joint names

        :return: List with joint names
        :rtype: list
        """
        jointNames = []
        for motor in self._robot.motors:
            if "virtualhand" not in motor.name:
                jointNames += [motor.name]
        return jointNames

    def getSensorNames(self):
        """
        Returns all sensor names

        :return: List with sensor names
        :rtype: list
        """
        sensorNames = []
        for sensor in self._robot.sensors:
            sensorNames += [sensor.name]
        return sensorNames

    def getAngleUpperLimit(self, jointName):
        """
        Returns the upper angle limit of a joint (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :return: Upper angle limit of the joint (degree)
        :rtype: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            return motor.upper_limit
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return 0.0

    def setAngleUpperLimit(self, jointName, angle):
        """
        Sets the upper angle limit of a joint (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :param angle: Angle (in degree)
        :type angle: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            motor.upper_limit = angle
        else:
            self._logger.warning('No joint "%s" found' % jointName)

    def getAngleLowerLimit(self, jointName):
        """
        Returns the lower angle limit of a joint (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :return: Lower angle limit of the joint (degree)
        :rtype: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            return motor.lower_limit
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return 0.0

    def setAngleLowerLimit(self, jointName, angle):
        """
        Sets the lower angle limit of a joint (in degree)

        :param jointName: Name of the joint
        :type jointName: str
        :param angle: Angle (in degree)
        :type angle: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            motor.lower_limit = angle
        else:
            self._logger.warning('No joint "%s" found' % jointName)

    def getTorqueLimit(self, jointName):
        """
        Returns the torque limit of a joint

        :param jointName: Name of the joint
        :type jointName: str
        :return: Torque limit of the joint
        :rtype: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            return motor.torque_limit
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return 0.0

    def getTemperature(self, jointName):
        """
        Returns the current temperature of a motor

        :param jointName: Name of the joint
        :type jointName: str
        :return: Temperature of the joint
        :rtype: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, "present_temperature"):
                return motor.present_temperature
            else:
                self._logger.warning("Joint %s has no present temperature" % jointName)
                return 0.0
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return 0.0

    def getCurrent(self, jointName):
        """
        Returns the current current of a motor

        :param jointName: Name of the joint
        :type jointName: str
        :return: Current of the joint
        :rtype: float
        """
        if hasattr(self._robot, jointName):
            if hasattr(self, "_leftHand") and self._leftHand.isHandMotor(jointName):
                return self._leftHand.getPresentCurrent(jointName)
            elif hasattr(self, "_rightHand") and self._rightHand.isHandMotor(jointName):
                return self._rightHand.getPresentCurrent(jointName)
            else:
                motor = getattr(self._robot, jointName)
                if hasattr(motor, "present_current"):
                    return motor.present_current
                else:
                    self._logger.warning("Joint %s has no present current" % jointName)
                    return 0.0
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return 0.0

    def getSpeed(self, jointName):
        """
        Returns the current speed of a motor

        :param jointName: Name of the joint
        :type jointName: str
        :return: Speed of the joint
        :rtype: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, "present_speed"):
                return motor.present_speed
            else:
                self._logger.warning("Joint %s has no present speed" % jointName)
                return 0.0
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return 0.0

    def setMaximumSpeed(self, maximumSpeed):
        """
        Sets the maximum allowed speed (in fraction of maximum possible speed).
        When giving a higher speed to any other
        functions the movement won't go over the value set here

        :param maximumSpeed: Maximum allowed speed (0 <= maximumSpeed <= 1.0)
        """
        if not 0.0 <= maximumSpeed <= 1.0:
            self._logger.warning("New maximum speed out of bounds (%d)" % maximumSpeed)
            return
        self._maximumSpeed = maximumSpeed

    def setStiffness(self, jointName, stiffness):
        """
        Sets the stiffness (0 <= stiffness <= 1) for a single motor

        :param jointName: Name of the joint
        :type jointName: str
        :param stiffness: Target stiffness
        :type stiffness: float
        """
        if not 0.0 <= stiffness <= 1.0:
            self._logger.warning("New stiffness out of bounds (%d)", stiffness)
            return

        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, "torque_limit"):
                motor.torque_limit = 100.0 * stiffness
            else:
                self._logger.warning("Joint %s has no torque limit" % jointName)

            if stiffness < 0.001:
                self.disableTorque(jointName)
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return

    def getStiffness(self, jointName):
        """
        Returns the current stiffness of a motor

        :param jointName: Name of the joint
        :type jointName: str
        :return: Stiffness of the joint
        :rtype: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if motor.compliant:  # no torque
                return 0.0
            elif hasattr(motor, "torque_limit"):
                return motor.torque_limit / 100.0
            else:
                self._logger.warning("Joint %s has no torque limit" % jointName)
                return 1.0
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return 0.0

    def setPID(self, jointName, p, i, d):
        """
        Sets the PID controller for a single motor. For more information see
        http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-64at_ar.htm#Actuator_Address_1A

        :param jointName: Name of the joint
        :type jointName: str
        :param p: Proportional band
        :type p: float
        :param i: Integral action
        :type i: float
        :param d: Derivative action
        :type d: float
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, "pid"):
                if (
                    hasattr(self, "_leftHand") and self._leftHand.isHandMotor(jointName)
                ) or (
                    hasattr(self, "_rightHand")
                    and self._rightHand.isHandMotor(jointName)
                ):
                    motor.pid_lock = False
                motor.pid = (p, i, d)
            else:
                self._logger.warning("Joint %s has no pid" % jointName)
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return

    def getPID(self, jointName):
        """
        Returns the current stifftness of a motor. For more information see
        http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-64at_ar.htm#Actuator_Address_1A

        :param jointName: Name of the joint
        :type jointName: str
        :return: Tupel: p,i,d
        :rtype: tuple
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, "pid"):
                return motor.pid
            else:
                self._logger.warning("Joint %s has no pid" % jointName)
                return (0.0, 0.0, 0.0)
        else:
            self._logger.warning('No joint "%s" found' % jointName)
            return (0.0, 0.0, 0.0)

    def enableTorqueAll(self):
        """
        Enables toruqe on all joints
        """
        for motor in self._robot.motors:
            motor.compliant = False

    def disableTorqueAll(self):
        """
        Disables toruqe on all joints
        """
        for motor in self._robot.motors:
            motor.compliant = True

    def enableTorque(self, jointName):
        """
        Enables torque on a single joint.

        :param jointName: Name of the motor
        :type jointName: str
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            motor.compliant = False
        else:
            self._logger.warning('No joint "%s" found' % jointName)

    def disableTorque(self, jointName):
        """
        Disables torque on a single joint.

        :param jointName: Name of the motor
        :type jointName: str
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            motor.compliant = True
        else:
            self._logger.warning('No joint "%s" found' % jointName)

    def getPose(self, objectName, relativeToObject=None):
        """
          Returns the current pose of the scene object with given name relative
          to the second given object

          :param objectName: Name of the object
          :type objectName: str
          :param relativeToObject: Name of the object that the position should
                                   be relative to
          :type relativeToObject: str
          :return: Position of the requestet object in x,y,z coordinates
                   relative to the second object
          :rtype: list of three floats
          """
        return self._robot.get_object_position(objectName, relativeToObject)

    def cleanup(self):
        """
        Cleans up the current connection to the robot. After this you can no
        longer control the robot
        """
        if self._robot is None:
            self._logger.warning("Cleanup called - but robot is not initialised")
            return

        self._logger.info("Closing robot connection")
        self._robot.close()
        self._robot = None

    def __del__(self):
        """
        Destructor
        """
        if self._robot is not None:
            self.cleanup()
