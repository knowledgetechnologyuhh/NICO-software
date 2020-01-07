#!/usr/bin/env python

from __future__ import print_function

import ast
import copy
import json
import logging
import math
import sys
import time
from collections import OrderedDict

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import nicomsg.msg
import nicomsg.srv
import roslaunch
import rospkg
import rospy
import rosservice
import sensor_msgs.msg
import std_msgs.msg
from kinematics.msg import FloatList
from kinematics.srv import FK_request, IK_request, collision_check

predefinedOrientations = dict()
predefinedOrientations["leftArm"] = dict()
predefinedOrientations["leftArm"]["sideGrasp"] = dict()
predefinedOrientations["leftArm"]["sideGrasp"]["x"] = 0.699255043834
predefinedOrientations["leftArm"]["sideGrasp"]["y"] = -0.0896847733213
predefinedOrientations["leftArm"]["sideGrasp"]["z"] = 0.705730390728
predefinedOrientations["leftArm"]["sideGrasp"]["w"] = 0.0703110283715
predefinedOrientations["leftArm"]["topGrasp"] = dict()
predefinedOrientations["leftArm"]["topGrasp"]["x"] = -0.50723004877
predefinedOrientations["leftArm"]["topGrasp"]["y"] = 0.522470812582
predefinedOrientations["leftArm"]["topGrasp"]["z"] = -0.503524230439
predefinedOrientations["leftArm"]["topGrasp"]["w"] = 0.464978791973
predefinedOrientations["rightArm"] = dict()
predefinedOrientations["rightArm"]["sideGrasp"] = dict()
predefinedOrientations["rightArm"]["sideGrasp"]["x"] = 0.0672353959324
predefinedOrientations["rightArm"]["sideGrasp"]["y"] = -0.705372281183
predefinedOrientations["rightArm"]["sideGrasp"]["z"] = 0.0754525764969
predefinedOrientations["rightArm"]["sideGrasp"]["w"] = 0.701595506807
predefinedOrientations["rightArm"]["topGrasp"] = dict()
predefinedOrientations["rightArm"]["topGrasp"]["x"] = -0.466315450108
predefinedOrientations["rightArm"]["topGrasp"]["y"] = -0.493845877759
predefinedOrientations["rightArm"]["topGrasp"]["z"] = 0.548284810402
predefinedOrientations["rightArm"]["topGrasp"]["w"] = 0.487903593646


class groupHandle:
    """
    The groupHandle class provides a high level interface for path planning.
    Plans are given for and executed on certain groups of joints
    """

    def __init__(
        self,
        groupName,
        vrep=True,
        robotMotorFile=None,
        vrepScene=None,
        kinematicsOnly=False,
        monitorPathExecution=None,
        visualize=False,
        rosnicoPrefix="/nico/motion",
        jointStateName="/joint_states",
    ):
        """
        :param groupName: Name of the planning group that should be moved.
                          Possible movement groups to use are:
                          'leftArm', 'rightArm', 'leftLeg', 'rightLeg'
        :type groupName: str
        :param vrep: Should movements be done in V-Rep simulation or on the real robot
        :type vrep: boolean
        :param robotMotorFile: Motor configuration file in json format
        :type robotMotorFile: str
        :param vrepScene: V-Rep scene to load if simulation is used
        :type vrepScene: str
        :param kinematicsOnly: Do not perform movements in simulated or real robot
        :type kinematicsOnly: boolean
        :param monitorPathExecution: Do or do not monitor if motion plans are correctly executed
        :type monitorPathExecution: boolean
        :param visualize: Visualize internal MoveIt! state in R-Viz
        :type visualize: boolean
        :param rosnicoPrefix: Topic prefix for motion handling in ROS
        :type rosnicoPrefix: str
        :param jointStateName: ROS topic for joint state information
        :type jointStateName: str
        """
        self.vrep = vrep
        self.rosnicoPrefix = rosnicoPrefix
        rospy.init_node("moveitWrapper", anonymous=True)

        # find packages
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        rospack = rospkg.RosPack()
        nicoros_pkg = rospack.get_path("nicoros")

        service_list = rosservice.get_service_list()
        # checking if nicomotion server is already running
        nicomotion = False
        if rosnicoPrefix + "/getAngle" in service_list:
            nicomotion = True
        # checking if moveit server is already running
        nicomoveit = False
        if "/plan_kinematic_path" in service_list:
            nicomoveit = True
            if not nicomotion and not kinematicsOnly:
                print(
                    "Please do not launch MoveIt! in demo mode, if you want to perform path planning!"
                )
                return

        # checking if MoveIt! is running and launch if not
        if not nicomoveit:
            if robotMotorFile is None:
                print(
                    "Please provide a motor configuration file or start rosnico with moveit capabilities"
                )
            elif vrep is True and vrepScene is None and not kinematicsOnly:
                print(
                    "You want to use V-Rep, but did not provide a V-Rep scene name. Please do that or start rosnico with moveit capabilities"
                )
            else:
                print("Starting MoveIt! service")
                if kinematicsOnly:
                    if visualize:
                        self.launch = roslaunch.parent.ROSLaunchParent(
                            uuid,
                            [nicoros_pkg + "/launch/nicomoveit_demo_visual.launch"],
                        )
                    else:
                        self.launch = roslaunch.parent.ROSLaunchParent(
                            uuid, [nicoros_pkg + "/launch/nicomoveit_demo.launch"]
                        )
                else:
                    rospy.set_param(
                        rosnicoPrefix + "/robotMotorFile",
                        nicoros_pkg + "/../../../json/" + robotMotorFile,
                    )
                    rospy.set_param(rosnicoPrefix + "/vrep", vrep)
                    rospy.set_param(
                        rosnicoPrefix + "/vrepScene",
                        nicoros_pkg + "/../../../v-rep/" + vrepScene,
                    )

                    if visualize:
                        self.launch = roslaunch.parent.ROSLaunchParent(
                            uuid, [nicoros_pkg + "/launch/nicoros_moveit_visual.launch"]
                        )
                    else:
                        self.launch = roslaunch.parent.ROSLaunchParent(
                            uuid, [nicoros_pkg + "/launch/nicoros_moveit.launch"]
                        )
                self.launch.start()
                rospy.wait_for_service("/plan_kinematic_path")

        if not kinematicsOnly:
            logging.info("Waiting for rosnico")
            rospy.wait_for_service("%s/getAngle" % rosnicoPrefix)

            logging.debug("Init publishers")
            self._setAngle = rospy.Publisher(
                "%s/setAngle" % rosnicoPrefix, nicomsg.msg.sff, queue_size=10
            )

            logging.debug("Init service proxies")
            self._getAngle = rospy.ServiceProxy(
                "%s/getAngle" % rosnicoPrefix, nicomsg.srv.GetValue
            )
            self._getJointNames = rospy.ServiceProxy(
                "%s/getJointNames" % rosnicoPrefix, nicomsg.srv.GetNames
            )
            self._getConfig = rospy.ServiceProxy(
                "%s/getConfig" % rosnicoPrefix, nicomsg.srv.GetString
            )
            self._getPose = rospy.ServiceProxy(
                "%s/getPose" % rosnicoPrefix, nicomsg.srv.GetValues
            )

            # get json configuration
            self.jsonConfig = ast.literal_eval(self._getConfig().data)
        else:
            with open(
                nicoros_pkg + "/../../../json/" + robotMotorFile, "r"
            ) as config_file:
                self.jsonConfig = json.load(config_file)

        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.groupName = groupName
        self.group = moveit_commander.MoveGroupCommander(groupName)
        # set default position tolerance to 1cm
        self.group.set_goal_position_tolerance(0.01)
        # set default orientation tolerance to 0.1 radians ~ 5.7 degrees
        self.group.set_goal_orientation_tolerance(0.1)
        # set default movement speed of joints
        rospy.set_param(rosnicoPrefix + "/fractionMaxSpeed", 0.1)
        if monitorPathExecution is not None:
            rospy.set_param(rosnicoPrefix + "/fakeExecution", not monitorPathExecution)
        self.defaultJointValues = self.group.get_current_joint_values()
        self.defaultPose = self.group.get_current_pose()
        self.planningTime = None
        self.executionTime = None

    def __exit__(self):
        self.launch.shutdown()

    def __planAndExecute(self, plan=None):
        """
        Internal function to execute path planning to a previously defined pose. If a plan has successfully been found it will be executed.

        :param plan: In case the planning was already done it can be commited with this parameter.
        :type plan: a RobotTrajectory
        """
        if plan == None:
            plan = self.group.plan()
        if plan.joint_trajectory.points == []:
            print("No valid plan has been found")
            return None
        # If a plan has been found execute the plan and return a list of points on the path
        print("Plan has been found")
        self.group.execute(plan)

    def getROSAngles(self):
        """
        Prints the current joint angles of NICO as they are represented in ROS
        :return: Current NICO joint angles in ROS
        :rtype: dictionary
        """
        dic = OrderedDict(
            zip(self.group.get_active_joints(), self.group.get_current_joint_values())
        )
        return dic

    def getNICOAngles(self, convertToMoveitAngles=False):
        """
        Prints the current joint angles of NICO in vrep or from the real robot

        :param convertToMoveitAngles: If set to True the angles are converted to the values that
                                      are used for ROS
        :type convertToMoveitAngles: boolean
        :return: Current Vrep joint angles
        :rtype: dictionary
        """
        dic = {}
        joints = self.group.get_active_joints()
        for joint in joints:
            angle = self._getAngle(joint).value
            if convertToMoveitAngles:
                angle = nicoToRosAngle(joint, angle, self.jsonConfig, self.vrep)
            dic[joint] = angle
        return dic

    def setMaxSpeed(self, fractionMaxSpeed):
        """
        Defines the movement speed of joints that is used if movement is executed on the real
        or simulated robot

        :param fractionMaxSpeed: movement speed of joints as fraction of maximum velocity
        :type fractionMaxSpeed: float
        """
        rospy.set_param(self.rosnicoPrefix + "/fractionMaxSpeed", fractionMaxSpeed)

    def getMaxSpeed(self):
        """
        Returns the movement speed of joints that is used if movement is executed on the real
        or simulated robot

        :return: movement speed of joints as fraction of maximum velocity
        :rtype: float
        """
        return rospy.get_param(self.rosnicoPrefix + "/fractionMaxSpeed")

    def setMotionPlanner(self, plannerID):
        """
        Specify which planner to use when motion planning
        Planning libraries: http://moveit.ros.org/documentation/planners/
        Planners from OMPL: http://ompl.kavrakilab.org/planners.html

        :param plannerID: The name of the planner to use
        :type plannerID: string
        """
        self.group.set_planner_id(plannerID)

    def setPathTolerance(self, pathTolerance):
        """
        Defines the maximum joint angle that the real/or simulated joint is allowed to
        deviate from the path plan. Set this value to enable automated stopping of motion
        in case of a potential collision. Be careful, a strict path tolerance will result
        in NICO stopping movement even in harmless situations

        :param pathTolerance: maximum allowed joint angle deviation in degree
        :type pathTolerance: float
        """
        rospy.set_param(
            self.rosnicoPrefix + "/pathTolerance", math.radians(pathTolerance)
        )

    def getPathTolerance(self):
        """
        Returns path tolerance currently used during path execution

        :return pathTolerance: maximum allowed joint angle deviation in degree
        :type pathTolerance: float
        """
        return math.degrees(rospy.get_param(self.rosnicoPrefix + "/pathTolerance", 0))

    def setPositionTolerance(self, positionTolerance):
        """
        Defines the area around the target that is acceptable as final position as
        a sphere around the target origin of the end-effector

        :param positionTolerance: radius of the sphere in meters
        :type positionTolerance: float
        """
        self.group.set_goal_position_tolerance(positionTolerance)

    def monitorPathExecution(self, monitorPathExecution):
        """
        Sets the ROS parameter that defines if motion plans are monitored to be correctly executed

        :param monitorPathExecution: should motion plan execution be monitored?
        :type monitorPathExecution: boolean
        """
        rospy.set_param(rosnicoPrefix + "/fakeExecution", not monitorPathExecution)

    def getPositionTolerance(self):
        """
        Returns the radius of the sphere around the target origin of the
        end-effector that defines the area that is acceptable as a final position

        :return: Position tolerance in meters
        :rtype: float
        """
        return self.group.get_goal_position_tolerance()

    def setOrientationTolerance(self, orientationTolerance):
        """
        Defines the allowed angle distance (roll, pitch, yaw) to the target origin of the end-effector

        :param orientationTolerance: allowed angle distance to the target in all orientations in degrees
        :type orientationTolerance: float
        """
        self.group.set_goal_orientation_tolerance(math.radians(orientationTolerance))

    def getOrientationTolerance(self):
        """
        Returns the allowed angle distance (roll, pitch, yaw) to the target origin of the end-effector

        :return: Orientation tolerance in degrees
        :rtype: float
        """
        return math.degrees(self.group.get_goal_orientation_tolerance())

    def setPlanningGroup(self, groupName):
        """
        Defines the current planning group that is used for all path planning and kinematic capabilities.

        :param groupName: Name of the planning group that should be moved.
                          Possible movement groups to use are:
                          'leftArm', 'rightArm', 'leftLeg', 'rightLeg'
        :type groupName: str
        """
        self.groupName = groupName
        self.group = moveit_commander.MoveGroupCommander(groupName)

    def getPlanningGroup(self, groupName):
        """
        Returns the name of the current planning group.
        """
        return self.groupName

    def computeIK(
        self,
        position,
        orientation,
        IKsolver=None,
        tip=None,
        ignoreCollisions=False,
        initialJointValues=[0, 0, 0, 0, 0, 0],
    ):
        """
        Computes inverse kinematics and return the corresponding joint angles

        :param position: A list with 3 parameters: [p_x,p_y,p_z]
                         Cartesian coordinates of the goal position
        :type position: list of floats
        :param orientation: A list with 4 parameters: [o_x,o_y,o_z,o_w]:
                            Goal orientation quaternion
                            or one of the predefined orientations (sideGrasp, topGrasp)
        :param IKsolver: Set the IK solver to use for the computation.
                         Default is the solver specified in the kinematics.yaml
                         Possible solvers: KDL, Trac-IK, BIO-IK
                         Trac-IK and BIO-IK have to be installed beforehand (see wiki)
        :type IKsolver: string
        :param tip: Frame (link) of the planning group that should reach the provided pose
        :type tip: string
        :param ignoreCollisions: Is a collision state allowed for the solution?
        :type ignoreCollisions:  boolean
        :param initialJointValues: A list with one float value for each joint of the planning group.
                                    Default starting value is 0 for each joint.
        :type initialJointValues: list of floats
        :return: Joint angles corresponding to request
                 or None if no solution could be found or if the service is not available
        :rtype: list of floats
        """
        if IKsolver is not None:
            if IKsolver == "KDL" or IKsolver == "kdl":
                rospy.set_param(
                    "/robot_description_kinematics/"
                    + self.groupName
                    + "/kinematics_solver",
                    "kdl_kinematics_plugin/KDLKinematicsPlugin",
                )
            elif (
                IKsolver == "trac_ik"
                or IKsolver == "TRAC_IK"
                or IKsolver == "trac"
                or IKsolver == "TRAC"
            ):
                rospy.set_param(
                    "/robot_description_kinematics/"
                    + self.groupName
                    + "/kinematics_solver",
                    "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
                )
            elif (
                IKsolver == "bio_ik"
                or IKsolver == "BIO_IK"
                or IKsolver == "bio"
                or IKsolver == "BIO"
            ):
                rospy.set_param(
                    "/robot_description_kinematics/"
                    + self.groupName
                    + "/kinematics_solver",
                    "bio_ik/BioIKKinematicsPlugin",
                )
            else:
                print("Please use either KDL, trac_ik or bio_ik as kinematic solvers")

        jointNames = self.group.get_active_joints()
        if type(initialJointValues) == list:
            initialJointValues = dict(zip(jointNames, initialJointValues))
        # Get current joint values
        dic = OrderedDict(zip(jointNames, self.group.get_current_joint_values()))
        # Set goal joint values
        for key in initialJointValues:
            dic[key] = nicoToRosAngle(
                key, initialJointValues[key], self.jsonConfig, self.vrep
            )

        initialJointValues = dic.values()

        service_list = rosservice.get_service_list()
        if "/moveit/compute_ik" not in service_list:
            print("Error: /moveit/compute_ik service is not available")
            return None

        if tip is None:
            if self.groupName == "leftArm":
                tip = "left_palm:11"
            if self.groupName == "rightArm":
                tip = "right_palm:11"
            if self.groupName == "leftLeg":
                tip = "left_foot:11"
            if self.groupName == "rightLeg":
                tip = "right_foot:11"

        pose = geometry_msgs.msg.Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        if type(orientation) is str:
            o_x = predefinedOrientations[self.groupName][orientation]["x"]
            o_y = predefinedOrientations[self.groupName][orientation]["y"]
            o_z = predefinedOrientations[self.groupName][orientation]["z"]
            o_w = predefinedOrientations[self.groupName][orientation]["w"]
        else:
            o_x = orientation[0]
            o_y = orientation[1]
            o_z = orientation[2]
            o_w = orientation[3]

        pose.orientation.x = o_x
        pose.orientation.y = o_y
        pose.orientation.z = o_z
        pose.orientation.w = o_w

        groupName = std_msgs.msg.String()
        groupName.data = self.groupName

        tipName = std_msgs.msg.String()
        tipName.data = tip

        ignoreCollisionsMsg = std_msgs.msg.Bool()
        ignoreCollisionsMsg.data = ignoreCollisions

        initialJointValuesMsg = FloatList()
        initialJointValuesMsg.data = []

        for jointValue in initialJointValues:
            floatMsg = std_msgs.msg.Float32()
            floatMsg.data = jointValue
            initialJointValuesMsg.data.append(floatMsg)

        try:
            ik_service = rospy.ServiceProxy("moveit/compute_ik", IK_request)
            rsp = ik_service(
                groupName, tipName, pose, ignoreCollisionsMsg, initialJointValuesMsg
            )
            if rsp.found_solution.data:
                print("Found IK solution")
                joint_angles = []
                for jointIdx in range(0, len(rsp.joint_angles)):
                    joint_angles.append(
                        rosToNicoAngle(
                            jointNames[jointIdx],
                            rsp.joint_angles[jointIdx].data,
                            self.jsonConfig,
                            self.vrep,
                        )
                    )
                print("Joint angles: ")
                print(joint_angles)
                return joint_angles
            else:
                print("No IK solution found")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        return None

    def computeFK(self, jointCoordinates, tip=None):
        """
        Computes inverse kinematics and return the corresponding joint angles

        :param jointCoordinates: List with the joint value for each joint in the correct order,
                           or a dictionary mapping joint names to joint values
                           (the default value for joints that are not included in the mapping is their current value)
        :type jointCoordinates: list of floats or dict mapping str to float
        :param tip: Frame (link) of the planning group for which the pose for the given joint values is requested
        :type tip: string
        :return: A list containing the position and a list containing the rotation as quaternion
                 or None if the service is not available
        :rtype: two lists of floats
        """
        if type(jointCoordinates) == list:
            jointCoordinates = dict(
                zip(self.group.get_active_joints(), jointCoordinates)
            )
        # Get current joint values
        dic = OrderedDict(
            zip(self.group.get_active_joints(), self.group.get_current_joint_values())
        )
        # Set goal joint values
        for key in jointCoordinates:
            dic[key] = nicoToRosAngle(
                key, jointCoordinates[key], self.jsonConfig, self.vrep
            )

        jointCoordinates = dic.values()

        service_list = rosservice.get_service_list()
        if "/moveit/compute_fk" not in service_list:
            print("Error: /moveit/compute_fk service is not available")
            return None

        if tip is None:
            if self.groupName == "leftArm":
                tip = "left_palm:11"
            if self.groupName == "rightArm":
                tip = "right_palm:11"
            if self.groupName == "leftLeg":
                tip = "left_foot:11"
            if self.groupName == "rightLeg":
                tip = "right_foot:11"

        jointValues = FloatList()
        jointValues.data = []

        for jointValue in jointCoordinates:
            floatMsg = std_msgs.msg.Float32()
            floatMsg.data = jointValue
            jointValues.data.append(floatMsg)

        groupName = std_msgs.msg.String()
        groupName.data = self.groupName

        tipName = std_msgs.msg.String()
        tipName.data = tip

        try:
            ik_service = rospy.ServiceProxy("moveit/compute_fk", FK_request)
            rsp = ik_service(groupName, tipName, jointValues)
            print("Found FK solution")
            pose = rsp.pose
            position = []
            position.append(pose.position.x)
            position.append(pose.position.y)
            position.append(pose.position.z)

            orientation = []
            orientation.append(pose.orientation.x)
            orientation.append(pose.orientation.y)
            orientation.append(pose.orientation.z)
            orientation.append(pose.orientation.w)

            print("Position: ")
            print(position)
            print("Orientation: ")
            print(orientation)

            return position, orientation
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
        return None

    def moveToPose(self, position, orientation):
        """
        Plan a path for the endeffector to the given position and orientation

        :param position: A list with 3 parameters: [p_x,p_y,p_z]
                         Cartesian coordinates of the goal position
        :type position: list of floats
        :param orientation: A list with 4 parameters: [o_x,o_y,o_z,o_w]:
                            Goal orientation quaternion
                            or one of the predefined orientations (sideGrasp, topGrasp)
        :type orientation: list of floats or string
        :return: List of points on the planned path or None if planning was not successful.
                 Each point is a list with one joint parameter for each joint of the planning group
        :rtype: list of lists of floats
        """
        print(
            "Trying to reach position",
            "x =",
            position[0],
            "y =",
            position[1],
            "z =",
            position[2],
            "with group",
            self.groupName,
        )
        if type(orientation) is str:
            o_x = predefinedOrientations[self.groupName][orientation]["x"]
            o_y = predefinedOrientations[self.groupName][orientation]["y"]
            o_z = predefinedOrientations[self.groupName][orientation]["z"]
            o_w = predefinedOrientations[self.groupName][orientation]["w"]
        else:
            o_x = orientation[0]
            o_y = orientation[1]
            o_z = orientation[2]
            o_w = orientation[3]
        # Define goal pose
        poseTarget = geometry_msgs.msg.Pose()
        poseTarget.position.x = position[0]
        poseTarget.position.y = position[1]
        poseTarget.position.z = position[2]
        poseTarget.orientation.x = o_x
        poseTarget.orientation.y = o_y
        poseTarget.orientation.z = o_z
        poseTarget.orientation.w = o_w
        # Set goal pose
        self.group.clear_pose_targets()
        self.group.set_pose_target(poseTarget)
        # Plan to goal pose and execute plan if found
        return self.__planAndExecute()

    def moveToPosition(self, position):
        """
        Plan a path for the endeffector to the given position.
        The orientation of the endeffector will be arbitrary

        :param position: A list with 3 parameters: [p_x,p_y,p_z]
                         Cartesian coordinates of the goal position
        :type position: list of floats
        :return: List of points on the planned path or None if planning was not successful.
                 Each point is a list with one joint parameter for each joint of the planning group
        :rtype: list of lists of floats
        """
        print(
            "Trying to reach position",
            "x =",
            position[0],
            "y =",
            position[1],
            "z =",
            position[2],
            "with group",
            self.groupName,
        )
        # Set goal pose
        self.group.clear_pose_targets()
        self.group.set_position_target(position)
        # Plan to goal pose and execute plan if found
        return self.__planAndExecute()

    def moveToJointCoordinates(self, jointCoordinates):
        """
        Plan to a robot configuration with the given joint values

        :param jointCoordinates: List with the joint value for each joint in the correct order,
                           or a dictionary mapping joint names to joint values
                           (the default value for joints that are not included in the mapping is their current value)
        :type jointCoordinates: list of floats or dict mapping str to float
        :return: List of points on the planned path or None if planning was not successful.
                 Each point is a list with one joint parameter for each joint of the planning group
        :rtype: list of lists of floats
        """
        print(
            "Trying to reach joint coordinates",
            jointCoordinates,
            "with group",
            self.groupName,
        )
        if type(jointCoordinates) == list:
            jointCoordinates = dict(
                zip(self.group.get_active_joints(), jointCoordinates)
            )
        # Get current joint values
        dic = OrderedDict(
            zip(self.group.get_active_joints(), self.group.get_current_joint_values())
        )
        # Set goal joint values
        for key in jointCoordinates:
            dic[key] = nicoToRosAngle(
                key, jointCoordinates[key], self.jsonConfig, self.vrep
            )
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(dic.values())
        # Plan to goal pose and execute plan if found
        return self.__planAndExecute()

    def shiftPose(self, axis, value):
        """
        Shift the pose on a certain axis with the distance given

        :param axis: Axis for which the pose should be shifted (0..5: X, Y, Z, R, P, Y)
        :type axis: float
        :param value:	Distance to shift in meters respectively degrees
        :type value: float
        :return: List of points on the planned path or None if planning was not successful.
                 Each point is a list with one joint parameter for each joint of the planning group
        :rtype: list of lists of floats
        """
        options = {0: "X", 1: "Y", 2: "Z", 3: "Roll", 4: "Pitch", 5: "Yaw"}
        print(
            "Trying to shift axis",
            options[axis],
            "with value",
            value,
            "for group",
            self.groupName,
        )
        if axis in [3, 4, 5]:
            value = math.radians(value)
        self.group.shift_pose_target(axis, value)
        # Plan to goal pose and execute plan if found
        return self.__planAndExecute()

    def shiftPosition(self, axis, value):
        """
        Shift the position on a certain axis with the distance given.
        The orientation of the endeffector will be arbitrary

        :param axis: Axis for which the position should be shifted (0..2: X, Y, Z)
        :type axis: float
        :param value:	Distance to shift
        :type value: float
        :return: List of points on the planned path or None if planning was not successful.
                 Each point is a list with one joint parameter for each joint of the planning group
        :rtype: list of lists of floats
        """
        currentPose = self.group.get_current_pose()
        x = currentPose.pose.position.x
        y = currentPose.pose.position.y
        z = currentPose.pose.position.z
        if axis == 0:
            x = x + value
        elif axis == 1:
            y = y + value
        elif axis == 2:
            z = z + value
        else:
            print("Please use an axis between 0 and 2 (0 -> x, 1 -> y, 2 ->z)")
        return self.moveToPosition([x, y, z])

    def toSafePosition(self):
        """
        Moves the robot to its initial state of this session.
        In this state it should be safe to disable the force control.
        To receive a collision free motion trajectories use the corresponding moveitWrapper function instead.
        """
        # to be certain that a safe position is reached we first try to reach the initial pose
        self.group.set_pose_target(self.defaultPose)
        self.__planAndExecute()
        # then we try to reach the exact initial joint values, this, however, is often not possible
        self.moveToJointCoordinates(self.defaultJointValues)

    def computeCartesianPath(
        self, axis, value, sufficient=False, startPosition=None, startOrientation=None
    ):
        """
        WARNING: Do not rely on this function to work as you expect! Please test in safe environment beforehand!

        Shift the pose on a certain axis with the distance given.
        This path has the property of being a straight line in cartesian coordinates

        :param axis: Axis for which the pose should be shifted (0..2: X, Y, Z)
        :type axis: float
        :param value:	Distance to shift
        :type value: float
        :param sufficient: If set to True the plan will be executed even if less
                           than 99% of the distance to the goal can be followed
        :type sufficient: boolean
        :param startPosition: Start position for the movement. Default is the current position
        :type startPosition: list of three floats
        :param startOrientation: Start orientation for the movement. Default is the current orientation.
                                 Can be defined as quaternion
                                 or one of the predefined orientations (sideGrasp, topGrasp)
        :type startOrientation: list of four floats or str
        :return: List of points on the planned path or None if planning was not successful.
                 Each point is a list with one joint parameter for each joint of the planning group
        :rtype: list of lists of floats
        """
        waypoints = []
        # start with the current pose
        currentPose = self.group.get_current_pose().pose
        waypoints.append(currentPose)
        # create pose with desired goal position
        if startPosition == None:
            p_x = currentPose.position.x
            p_y = currentPose.position.y
            p_z = currentPose.position.z
        else:
            p_x = startPosition[0]
            p_y = startPosition[1]
            p_z = startPosition[2]
        if startOrientation == None:
            o_x = currentPose.orientation.x
            o_y = currentPose.orientation.y
            o_z = currentPose.orientation.z
            o_w = currentPose.orientation.w
        elif type(startOrientation) is str:
            o_x = predefinedOrientations[self.groupName][startOrientation]["x"]
            o_y = predefinedOrientations[self.groupName][startOrientation]["y"]
            o_z = predefinedOrientations[self.groupName][startOrientation]["z"]
            o_w = predefinedOrientations[self.groupName][startOrientation]["w"]
        else:
            o_x = startOrientation[0]
            o_y = startOrientation[1]
            o_z = startOrientation[2]
            o_w = startOrientation[3]
        poseTarget = geometry_msgs.msg.Pose()
        poseTarget.orientation.x = o_x
        poseTarget.orientation.y = o_y
        poseTarget.orientation.z = o_z
        poseTarget.orientation.w = o_w
        poseTarget.position.x = p_x
        poseTarget.position.y = p_y
        poseTarget.position.z = p_z
        if axis == 0:
            poseTarget.position.x = poseTarget.position.x + value
        elif axis == 1:
            poseTarget.position.y = poseTarget.position.y + value
        elif axis == 2:
            poseTarget.position.z = poseTarget.position.z + value
        else:
            print("Please use an axis between 0 and 2 (0 -> x, 1 -> y, 2 ->z)")
        # the second waypoint is our desired goal
        waypoints.append(copy.deepcopy(poseTarget))
        # the fraction contains information about the fraction of the path that was followed
        start = time.clock()
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.planningTime = time.clock() - start
        # usually distances of about 10cm are planned,
        # to reach the goal with 1mm tolerance at least 99% of the way should be followed
        if fraction < 0.99 and sufficient == False:
            print(
                "Only "
                + str(fraction * 100)
                + " percent of the catersian path can be followed,"
                + " if this is sufficient for you, call this method again with third argument 'True'"
            )
            return None
        else:
            return self.__planAndExecute(plan)

    def isColliding(self, jointCoordinates):
        """
        Check if the NICO is in collision with itself or the environment
        for a certain joint configuration

        :param jointCoordinates: List with the joint value for each joint in the correct order,
                           or a dictionary mapping joint names to joint values
                           (the default value for joints that are not included in the mapping is their current value)
        :type jointCoordinates: list of floats or dict mapping str to float
        :return: Returns True if the NICO is in collision at the given joint angles
                 and returns False otherwise
        :rtype: boolean
        """
        service_list = rosservice.get_service_list()
        if "/moveit/collision_check" not in service_list:
            print("Error: /moveit/collision_check service is not available")
            return None

        if type(jointCoordinates) == list:
            jointCoordinates = dict(
                zip(self.group.get_active_joints(), jointCoordinates)
            )
        # Get current joint values
        dic = OrderedDict(
            zip(self.group.get_active_joints(), self.group.get_current_joint_values())
        )
        for key in jointCoordinates:
            dic[key] = nicoToRosAngle(
                key, jointCoordinates[key], self.jsonConfig, self.vrep
            )

        jointCoordinates = dic.values()
        print(jointCoordinates)
        jointValues = FloatList()
        jointValues.data = []

        for jointValue in jointCoordinates:
            floatMsg = std_msgs.msg.Float32()
            floatMsg.data = jointValue
            jointValues.data.append(floatMsg)

        groupName = std_msgs.msg.String()
        groupName.data = self.groupName

        try:
            collision_check_service = rospy.ServiceProxy(
                "moveit/collision_check", collision_check
            )
            rsp = collision_check_service(groupName, jointValues)
            if rsp.collision_state.data:
                print(
                    "The planning group " + self.groupName + " is in a collision state"
                )
            else:
                print(
                    "The planning group "
                    + self.groupName
                    + " is not in a collision state"
                )
            return rsp.collision_state.data
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        return None

    def getRandomJointValues(self):
        """
        Function to generate random joint values for the motion group

        :return: Returns joint values for a random group pose
        :rtype: list of floats
        """
        randomPose = self.group.get_random_joint_values()
        joints = self.group.get_active_joints()
        # the joint values are given in MoveIt! reference system
        for jointIdx in range(0, len(randomPose)):
            randomPose[jointIdx] = rosToNicoAngle(
                joints[jointIdx], randomPose[jointIdx], self.jsonConfig, self.vrep
            )
        return randomPose

    def addBox(self, name, position, size):
        """
        Includes a box as an obstacle in the simulation environment.
        Planned paths should recognize obstacles.

        :param name: Name of the object that is added to the scene
        :type name: str
        :param position: A list with 3 parameters: [p_x,p_y,p_z]
                         Cartesian coordinates of the goal position
        :type position: list of floats
        :param size: A list with 3 parameters: [s_x,s_y,s_z]
                     Scaling factors for the size of the box in the scene
        :type size: list of floats
        """
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "/world"
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        self.scene.add_box(name, pose, size)

    def addMesh(self, name, fileName, position=[0, 0, 0], size=[1, 1, 1]):
        """
        Includes a mesh model as an obstacle in the simulation environment.
        Planned paths should recognize obstacles.
        To include a mesh from a vrep-scene, export the mesh

        :param name: Name of the object that is added to the scene
        :type name: str
        :param fileName: File with the mesh
        :type fileName: str
        :param position: A list with 3 parameters: [p_x,p_y,p_z]
                         Cartesian coordinates of the goal position
        :type position: list of floats
        :param size: A list with 3 parameters: [s_x,s_y,s_z]
                     Scaling factors for the size of the box in the scene
        :type size: list of floats
        """
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "/world"
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        self.scene.add_mesh(name, pose, fileName, size)

    def removeObject(self, name):
        """
        Removes an object from the MoveIt! scene

        :param name: Name of the object that is removed from the scene
        :type name: str
        """
        self.scene.remove_world_object(name)

    def __test(self):
        """
        Method for testing the MoveIt! functionalities.
        """

        self.group.set_planner_id("RRTConnectkConfigDefault")

        print("increase tolerance")
        self.group.set_goal_position_tolerance(0.1)
        self.group.set_goal_orientation_tolerance(0.2)

        values = [
            -0.7417967536653856,
            2.2122617473316843,
            0.3553677613749635,
            -1.4112707151742188,
            -1.3299268114123495,
            -0.3222468796948495,
        ]
        self.group.set_joint_value_target(values)
        print("plan")
        plan = self.group.plan()
        print("execute")
        self.group.execute(plan)
        rospy.sleep(2)

        print("Known constraints: ")
        print(self.group.get_known_constraints())

        for i in range(10):
            pose_target = self.group.get_random_pose()
            # print(pose_target)
            self.group.set_pose_target(pose_target)
            self.group.go()

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = 0.158434282046
        pose_target.orientation.y = 0.807295567431
        pose_target.orientation.z = 0.258856217432
        pose_target.orientation.w = 0.506128347138
        pose_target.position.x = 0.390251923048
        pose_target.position.y = 0.339816569177
        pose_target.position.z = 0.760846647956

        self.group.set_pose_target(pose_target)
        print("1")
        self.group.go()
        rospy.sleep(2)

        pose_target.orientation.x = -0.973379789458
        pose_target.orientation.y = -0.0139847757448
        pose_target.orientation.z = 0.00340166727638
        pose_target.orientation.w = 0.228745798172
        pose_target.position.x = 0.0718405144708
        pose_target.position.y = 0.153894236006
        pose_target.position.z = 0.395156039253

        self.group.set_pose_target(pose_target)
        print("2")
        self.group.go()

    def sittingPosition(self):
        """
        This function can be used to bring the simulated robot into a sitting pose.
        Does currently not work, because leg motors are not available.

        :param jointName: Name of the joint
        :type jointName: str
        :return: Angle of the joint (degree)
        :rtype: float
        """
        # [0.0601951067222 -0.084078543203 -1.53735464204 1.24698948964 0.305836083766 0.0223105563298]
        p_x = 0.27417440386
        p_y = 0.0576193877269
        p_z = 0.227848717945
        o_x = 0.0020579646314
        o_y = -0.00255815230453
        o_z = 2.94616341365e-05
        o_w = 0.999994609871
        oldPlanningGroup = self.getPlanningGroup()
        self.setPlanningGroup("leftLeg")
        self.moveToPose([p_x, p_y, p_z], [o_x, o_y, o_z, o_w])

        # [-0.031318849578 0.0614412972261 -1.50805089035 1.20882593629 0.387770525688 -0.0962579440558]
        p_x = 0.274107814509
        p_y = -0.0547849446914
        p_z = 0.227952902805
        o_x = -0.00276895075104
        o_y = -0.00276769274683
        o_z = -4.37108219655e-05
        o_w = 0.99999233541
        self.setPlanningGroup("rightLeg")
        self.moveToPose([p_x, p_y, p_z], [o_x, o_y, o_z, o_w])
        self.setPlanningGroup(oldPlanningGroup)


def rosToNicoAngle(jointName, value, jsonConfig=None, vrep=True):
    """
    Internal function to convert an angle from the MoveIt! simulation to an angle for the V-Rep simulation or the real robot

    :param jointName: Name of the joint of interest
    :type jointName: str
    :param value: Joint angle from MoveIt! (in radians) that should be converted to a corresponding joint angle in V-Rep (in degrees)
    :type value: float
    """
    value = math.degrees(value)
    if jsonConfig is not None:
        if jointName in jsonConfig[u"motors"]:
            # if jsonConfig[u"motors"][jointName][u"orientation"] == u"indirect":
            #     value = -value
            value = value - jsonConfig[u"motors"][jointName][u"offset"]
    return value


def nicoToRosAngle(jointName, value, jsonConfig=None, vrep=True):
    """
    Internal function to convert an angle from the V-Rep simulation or the real robot to an angle for the MoveIt! simulation

    :param jointName: Name of the joint of interest
    :type jointName: str
    :param value: Joint angle from V-Rep (in degrees) that should be converted to a corresponding joint angle in MoveIt! (in radians)
    :type value: float
    """
    print(jointName)
    if jsonConfig is not None:
        if jointName in jsonConfig[u"motors"]:
            value = value + jsonConfig[u"motors"][jointName][u"offset"]
            # if jsonConfig[u"motors"][jointName][u"orientation"] == u"indirect":
            #     value = -value
    return math.radians(value)
