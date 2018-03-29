import logging
import json
import re
import pprint
import collections

import pypot.robot
import pypot.vrep
from pypot.vrep.remoteApiBindings import vrep as remote_api

import _nicomotion_internal.hand
import _nicomotion_internal.RH7D_hand

class Motion:
    """
    The Motion class provides a high level interface to various movement related functions of the NICO robot
    """

    def __init__(self, motorConfig='config.json', vrep=False, vrepHost='127.0.0.1', vrepPort=19997, vrepScene=None):
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
        """
        self._robot = None
        self._maximumSpeed = 1.0
        self._vrep = vrep
        self._vrepIO = None
        self._handModel = "RH4D"

        with open(motorConfig, 'r') as config_file:
            config = json.load(config_file)

        self._config = config

        if vrep:
            logging.info('Using VREP')
            # TODO Remove the filtering of l_wrist_x once the new model is updated
            to_remove = ['l_virtualhand_x', 'r_virtualhand_x']
            for motor in to_remove:
                config['motors'].pop(motor)
                for group in config['motorgroups'].keys():
                    config['motorgroups'][group] = [x for x in config['motorgroups'][group] if x != motor]
            self._robot = pypot.vrep.from_vrep(config, vrepHost, vrepPort, vrepScene)
            self._vrepIO = self._robot._controllers[0].io
        else:
            logging.info('Using robot')
            try:
                self._robot = pypot.robot.from_config(config)
            except IndexError as e:
                regex = re.compile('.*\[.*\].*\[(?P<ids>.*)\].*')
                match = regex.match(e.message)
                string = match.group('ids')
                for id in string.split(','):
                    id = int(id)
                    for motor in config['motors'].keys():
                        if config['motors'][motor]['id'] == id:
                            logging.warning('Removing motor %s (%i)' %(motor, id))
                            config['motors'].pop(motor)
                            for group in config['motorgroups'].keys():
                                config['motorgroups'][group] = [x for x in config['motorgroups'][group] if x != motor]
                logging.warning('New config created:')
                logging.warning(pprint.pformat(config))
                self._robot = pypot.robot.from_config(config)
        if hasattr(self._robot, "r_middlefingers_x") or hasattr(self._robot, "l_middlefingers_x"):
            self._handModel = "RH7D"

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
        Starts the V-REP Simulation. If 'synchronize' is set True the simulation steps
        will not advance until nextSimulationStep() is called.

        :param synchronize: Enables control over simulation time steps
        :type synchronize: bool
        """
        if self._vrep:
            if synchronize:
                remote_api.simxSynchronous(self._vrepIO.client_id, True)
                self._vrepIO.start_simulation()
            else:
                remote_api.simxSynchronous(self._vrepIO.client_id, False)
                self._vrepIO.start_simulation()
        else:
            logging.warning('startSimulation() has no effect on a real robot')

    def setSimulationDeltatime(self, dt):
        """
        Sets the timeframe which one simulation step represents. Only works while the simulation is stopped and dt is set to custom in V-REP.

        :param dt: timeframe of one simulation step
        :type dt: int
        """
        if self._vrep:
            self._vrepIO.call_remote_api('simxSetFloatingParameter', remote_api.sim_floatparam_simulation_time_step, dt)
        else:
            logging.warning('nextSimulationStep() has no effect on a real robot')

    def nextSimulationStep(self):
        """
        Advances the V-REP simulation by one step if synchronize was set on startSimulation().
        """
        if self._vrep:
            remote_api.simxSynchronousTrigger(self._vrepIO.client_id)
        else:
            logging.warning('nextSimulationStep() has no effect on a real robot')

    def stopSimulation(self):
        """
        Stops the V-REP simulation
        """
        if self._vrep:
            self._vrepIO.stop_simulation()
        else:
            logging.warning('stopSimulation() has no effect on a real robot')

    def resetSimulation(self):
        """
        Restarts the V-REP simulation
        """
        if self._vrep:
            self._vrepIO.restart_simulation()
        else:
            logging.warning('resetSimulation() has no effect on a real robot')

    def callVREPRemoteApi(self, func_name, *args, **kwargs):
        """ Calls any remote API func in a thread_safe way.

        :param str func_name: name of the remote API func to call
        :param args: args to pass to the remote API call
        :param kwargs: args to pass to the remote API call

        :return: api response

        .. note:: You can add an extra keyword to specify if you want to use the streaming or sending mode. The oneshot_wait mode is used by default (see `here <http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#operationModes>`_ for details about possible modes).

        .. warning:: You should not pass the clientId and the operationMode as arguments. They will be automatically added.
        """
        if self._vrep:
            return self._vrepIO.call_remote_api(func_name, *args, **kwargs)
        else:
            logging.warning('callVREPRemoteApi() has no effect on a real robot')
            return None

    def getVrepIO(self):
        """
        Gives access to pypots vrep IO (see https://poppy-project.github.io/pypot/pypot.vrep.html#pypot.vrep.io.VrepIO)

        :return: Pypot vrep IO
        :rtype: pypot.vrep.io
        """
        if self._vrep:
            return self._vrepIO
        else:
            logging.warning('A real robot has no VREP controller')
            return None

    def thumbsUp(self, handName, fractionMaxSpeed=1.0):
        """
        Gives a thumbs up with the specified hand. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
        :type fractionMaxSpeed: float
        """
        if self._vrep:
            logging.warning("'Thumbs up' pose is not supported for vrep")
        else:
            if self._handModel=="RH7D":
                _nicomotion_internal.RH7D_hand.thumbsUp(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed))
            else:
                logging.warning("'Thumbs up' pose is not supported for hand model %s", self._handModel)

    def okSign(self, handName, fractionMaxSpeed=1.0):
        """
        Performs the 'OK' hand signal that divers use. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
        :type fractionMaxSpeed: float
        """
        if self._vrep:
            logging.warning("'OK' pose is not supported for vrep")
        else:
            if self._handModel=="RH7D":
                _nicomotion_internal.RH7D_hand.okSign(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed))
            else:
                logging.warning("'OK' pose is not supported for hand model %s", self._handModel)

    def pinchToIndex(self, handName, fractionMaxSpeed=1.0):
        """
        Pinches thumb and index finger together. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
        :type fractionMaxSpeed: float
        """
        if self._vrep:
            logging.warning("'pinch to index' pose is not supported for vrep")
        else:
            if self._handModel=="RH7D":
                _nicomotion_internal.RH7D_hand.pinchToIndex(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed))
            else:
                logging.warning("'pinch to index' pose is not supported for hand model %s", self._handModel)

    def pencilGrip(self, handName, fractionMaxSpeed=1.0):
        """
        Performs a grip able to hold a (thick) pencil. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
        :type fractionMaxSpeed: float
        """
        if self._vrep:
            logging.warning("'pencil grip' pose is not supported for vrep")
        else:
            if self._handModel=="RH7D":
                _nicomotion_internal.RH7D_hand.pencilGrip(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed))
            else:
                logging.warning("'pencil grip' pose is not supported for hand model %s", self._handModel)

    def keyGrip(self, handName, fractionMaxSpeed=1.0):
        """
        Performs a grip able to hold a key card. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
        :type fractionMaxSpeed: float
        """
        if self._vrep:
            logging.warning("'key grip' pose is not supported for vrep")
        else:
            if self._handModel=="RH7D":
                _nicomotion_internal.RH7D_hand.keyGrip(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed))
            else:
                logging.warning("'key grip' pose is not supported for hand model %s", self._handModel)

    def pointAt(self, handName, fractionMaxSpeed=1.0, percentage=1.0):
        """
        Sticks the indexfinger out to point at something. handName can be 'RHand' or 'LHand'

        :param handName: Name of the hand (RHand, LHand)
        :type handName: str
        :param fractionMaxSpeed: Speed at which hand should open. Default: 1.0
        :type fractionMaxSpeed: float
        """
        if self._vrep:
            logging.warning("'Point at' pose is not supported for vrep")
        else:
            if self._handModel=="RH7D":
                _nicomotion_internal.RH7D_hand.pointAt(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed))
            else:
                logging.warning("'Point at' pose is not supported for hand model %s", self._handModel)

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
        if self._vrep:
            _nicomotion_internal.hand.openHandVREP(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed), percentage)
        else:
            if self._handModel=="RH4D":
                _nicomotion_internal.hand.openHand(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed), percentage)
            else:
                if (percentage < 1.0):
                    logging.warning("Open hand for RH7D doesn't support percentage parameter")
                _nicomotion_internal.RH7D_hand.openHand(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed))

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
        if self._vrep:
            _nicomotion_internal.hand.closeHandVREP(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed), percentage)
        else:
            if self._handModel=="RH4D":
                _nicomotion_internal.hand.closeHand(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed), percentage)
            else:
                if percentage!=1.0:
                    logging.warning("Close hand for RH7D doesn't support percentage parameter")
                _nicomotion_internal.RH7D_hand.closeHand(self._robot, handName, min(fractionMaxSpeed, self._maximumSpeed))

    def enableForceControlAll(self, goalForce = 500):
        """
        Enables force control for all joints which support this feature

        :param goalForce: Goal force (0-2000)
        :type goalForce: int
        """
        for motor in self._robot.motors:
            if hasattr(motor, 'force_control_enable'):
                motor.force_control_enable = True
                motor.goal_force = goalForce

    def disableForceControlAll(self):
        """
        Disables force control for all joints which support this feature
        """
        for motor in self._robot.motors:
            if hasattr(motor, 'force_control_enable'):
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
            if hasattr(motor, 'force_control_enable'):
                motor.force_control_enable = True
                motor.goal_force = goalForce
            else:
                logging.warning('Joint %s has no force control' % jointName)
        else:
            logging.warning('No joint "%s" found' % jointName)
            return

    def disableForceControl(self, jointName):
        """
        Disables force control for a single joint

        :param jointName: Name of the joint
        :type jointName: str
        """
        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, 'force_control_enable'):
                motor.force_control_enable = False
            else:
                logging.warning('Joint %s has no force control' % jointName)
        else:
            logging.warning('No joint "%s" found' % jointName)
            return

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
            if self._handModel == "RH7D" and _nicomotion_internal.RH7D_hand.isHandMotor(jointName):
                _nicomotion_internal.RH7D_hand.setAngle(jointName, angle, fractionMaxSpeed)
            else:
                motor = getattr(self._robot, jointName)
                motor.compliant = False
                motor.goal_speed = 1000.0 * min(fractionMaxSpeed, self._maximumSpeed)
                motor.goal_position = angle
        else:
            logging.warning('No joint "%s" found' % jointName)
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
            motor = getattr(self._robot, jointName)
            if self._handModel == "RH7D" and _nicomotion_internal.RH7D_hand.isHandMotor(jointName):
                _nicomotion_internal.RH7D_hand.setAngle(jointName, change + motor.present_position, fractionMaxSpeed)
            else:
                motor.compliant = False
                motor.goal_speed = 1000.0 * min(fractionMaxSpeed, self._maximumSpeed)
                motor.goal_position = change + motor.present_position
        else:
            logging.warning('No joint "%s" found' % jointName)
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
            motor = getattr(self._robot, jointName)
            return motor.present_position
        else:
            logging.warning('No joint "%s" found' % jointName)
            return 0.0

    def getJointNames(self):
        """
        Returns all joint names

        :return: List with joint names
        :rtype: list
        """
        jointNames = []
        for motor in self._robot.motors:
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
            logging.warning('No joint "%s" found' % jointName)
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
            logging.warning('No joint "%s" found' % jointName)

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
            logging.warning('No joint "%s" found' % jointName)
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
            logging.warning('No joint "%s" found' % jointName)

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
            logging.warning('No joint "%s" found' % jointName)
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
            if hasattr(motor, 'present_temperature'):
                return motor.present_temperature
            else:
                logging.warning('Joint %s has no present temperature' % jointName)
                return 0.0
        else:
            logging.warning('No joint "%s" found' % jointName)
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
            if self._handModel == "RH4D" and _nicomotion_internal.hand.isHandMotor(jointName):
                return _nicomotion_internal.hand.getPresentCurrent(jointName)
            elif self._handModel == "RH7D" and _nicomotion_internal.RH7D_hand.isHandMotor(jointName):
                return _nicomotion_internal.RH7D_hand.getPresentCurrent
            else:
                motor = getattr(self._robot, jointName)
                if hasattr(motor, 'present_current'):
                    return motor.present_current
                else:
                    logging.warning('Joint %s has no present current' % jointName)
                    return 0.0
        else:
            logging.warning('No joint "%s" found' % jointName)
            return 0.0

    def setMaximumSpeed(self, maximumSpeed):
        """
        Sets the maximum allowed speed (in fraction of maximum possible speed). When giving a higher speed to any other
        functions the movement won't go over the value set here

        :param maximumSpeed: Maximum allowed speed (0 <= maximumSpeed <= 1.0)
        """
        if not 0.0 <= maximumSpeed <= 1.0:
            logging.warning('New maximum speed out of bounds (%d)' % maximumSpeed)
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
            logging.warning('New stiffness out of bounds (%d)' % maximumSpeed)
            return

        if hasattr(self._robot, jointName):
            motor = getattr(self._robot, jointName)
            if hasattr(motor, 'torque_limit'):
                motor.torque_limit = 100.0 * stiffness
            else:
                logging.warning('Joint %s has no torque limit' % jointName)

            if(stiffness < 0.001):
                self.disableTorque(jointName)
        else:
            logging.warning('No joint "%s" found' % jointName)
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
            if motor.compliant: # no torque
                return 0.0
            elif hasattr(motor, 'torque_limit'):
                return motor.torque_limit / 100.0
            else:
                logging.warning('Joint %s has no torque limit' % jointName)
                return 1.0
        else:
            logging.warning('No joint "%s" found' % jointName)
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
            if hasattr(motor, 'pid'):
                motor.pid = (p, i, d)
            else:
                logging.warning('Joint %s has no pid' % jointName)
        else:
            logging.warning('No joint "%s" found' % jointName)
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
            if hasattr(motor, 'pid'):
                return motor.pid
            else:
                logging.warning('Joint %s has no pid' % jointName)
                return (0.0, 0.0, 0.0)
        else:
            logging.warning('No joint "%s" found' % jointName)
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
            logging.warning('No joint "%s" found' % jointName)

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
            logging.warning('No joint "%s" found' % jointName)

    def getPose(self, objectName, relativeToObject=None):
      """
        Returns the current pose of the scene object with given name relative to the second given object

        :param objectName: Name of the object
        :type objectName: str
        :param relativeToObject: Name of the object that the position should be relative to
        :type relativeToObject: str
        :return: Position of the requestet object in x,y,z coordinates relative to the second object
        :rtype: list of three floats
        """
      return self._robot.get_object_position(objectName,relativeToObject)

    def cleanup(self):
        """
        Cleans up the current connection to the robot. After this you can no longer control the robot
        """
        if self._robot is None:
            logging.warning('Cleanup called - but robot is not initialised')
            return

        logging.info('Closing robot connection')
        self._robot.close()
        self._robot = None
        logging.shutdown()

    def __del__(self):
        """
        Destructor
        """
        if self._robot is  not None:
            self.cleanup()
