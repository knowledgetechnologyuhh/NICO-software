#!/usr/bin/env python

import argparse
import ast
import logging
import math

import actionlib
import control_msgs.msg
import nicomsg.msg
import nicomsg.srv
import rospy
import trajectory_msgs.msg
from nicomoveit import moveitWrapper


class TrajectoryServer:
    """
    The TrajectoryServer exposes the FollowJointTrajectoryAction through ROS.
    It uses the NICO ROS motion API for communication with the robot.
    """

    @staticmethod
    def getConfig():
        """
        Returns a default config dict

        :return: dict
        """
        return {
            "rostopicName": "/trajectory",
            "rosnicoPrefix": "/nico/motion",
            "check_correct_path_execution": True,
        }

    def __init__(self, planningGroup, config=None):
        """
        The TrajectoryServer provides the FollowJointTrajectoryAction via ROS

        :param config: Configuration
        :type config: dict
        """
        if config is None:
            config = TrajectoryServer.getConfig()

        self.config = config

        self._error_string = "Successful"
        # Error codes see
        # http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html
        self._error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
        self._path_tolerance = dict()

        logging.debug("Init ROS")
        rospy.init_node("nico_trajectory_server", anonymous=True)

        logging.info("Waiting for rosnico")
        rospy.wait_for_service("%s/getAngle" % config["rosnicoPrefix"])

        # at default the allowed joint angle error is 2 radians
        # setting this to a lower value will allow to stop motion after collision,
        # however because of the delayed motion on the NICO valid paths might not be correctly executed
        self.pathTolerance = 2
        # angle velocity for the movements as fraction of the maximum velocity
        self.fractionMaxSpeed = 0.1

        rostopic = planningGroup + config["rostopicName"]

        logging.debug("Init action server")
        # for group in config['planningGroups']:
        self._actionServer = actionlib.SimpleActionServer(
            "%s" % rostopic,
            control_msgs.msg.FollowJointTrajectoryAction,
            execute_cb=self._FollowJointTrajectoryCallback,
            auto_start=False,
        )
        self._actionServer.start()

        logging.debug("Init publishers")
        self._setAngle = rospy.Publisher(
            "%s/setAngle" % config["rosnicoPrefix"], nicomsg.msg.sff, queue_size=10
        )

        logging.debug("Init service proxies")
        self._getAngle = rospy.ServiceProxy(
            "%s/getAngle" % config["rosnicoPrefix"], nicomsg.srv.GetValue
        )
        self._getJointNames = rospy.ServiceProxy(
            "%s/getJointNames" % config["rosnicoPrefix"], nicomsg.srv.GetNames
        )
        self._getConfig = rospy.ServiceProxy(
            "%s/getConfig" % config["rosnicoPrefix"], nicomsg.srv.GetString
        )
        self.jsonConfig = ast.literal_eval(self._getConfig().data)
        self._getVrep = rospy.ServiceProxy(
            "%s/getVrep" % config["rosnicoPrefix"], nicomsg.srv.GetString
        )
        self.vrep = bool(self._getVrep().data)

    def _FollowJointTrajectoryCallback(self, message):
        """
        Callback for FollowJointTrajectoryAction

        :param message: FollowJointTrajectoryAction message
        :type message: control_msgs.msg.FollowJointTrajectoryAction
        """
        self._error_string = "Successful"
        self._error_code = control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL

        self.pathTolerance = rospy.get_param(
            config["rosnicoPrefix"] + "/pathTolerance", self.pathTolerance
        )
        self.fractionMaxSpeed = rospy.get_param(
            config["rosnicoPrefix"] + "/fractionMaxSpeed", self.fractionMaxSpeed
        )
        self.config["check_correct_path_execution"] = not rospy.get_param(
            config["rosnicoPrefix"] + "/fakeExecution",
            not self.config["check_correct_path_execution"],
        )

        self._testJoints(message.trajectory.joint_names)

        # rospy.logwarn(str(message))

        if (
            self._error_code
            is not control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
        ):
            self._sendResult()
            return

        self._path_tolerance = dict()

        for JointTolerance in message.path_tolerance:
            if JointTolerance.position is not -1 and JointTolerance.position is not 0:
                self._path_tolerance[JointTolerance.name] = JointTolerance.position

        start = message.trajectory.header.stamp

        for i in range(0, len(message.trajectory.points)):
            # Movement should start once the time in the stamp has been reached
            # See http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
            sleep = start - rospy.Time.now()
            if sleep.to_sec() > 0.0:
                rospy.sleep(sleep)
            self._moveTo(message.trajectory.joint_names, message.trajectory.points[i])
            sleep = message.trajectory.points[i].time_from_start - (
                rospy.Time.now() - start
            )
            if sleep.to_sec() > 0.0:
                rospy.sleep(sleep)
            self._sendFeedback(
                message.trajectory.joint_names, message.trajectory.points[i], start
            )
            if (
                self._error_code
                is not control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
            ):
                rospy.logwarn(str("ERROR occured"))
                self._moveToCurrentJointAngles(message.trajectory.joint_names)
                break
        if self.config["check_correct_path_execution"]:
            rospy.logwarn(str("check final position"))
            self._checkFinalPosition(message, start)
        else:
            rospy.logwarn(str("skip check of final position"))
        rospy.logwarn(str("sendResult"))
        self._sendResult()

    def _testJoints(self, jointNames):
        """
        Tests if the requested joints are known to the robot.

        If they are unknown, error_code is set.

        :param jointNames: Joint names used in the trajectory
        :type jointNames: list
        """
        names = frozenset(self._getJointNames().names)
        unknown = []
        for joint in jointNames:
            if not joint in names:
                unknown += [joint]

        if len(unknown) is not 0:
            self._error_code = (
                control_msgs.msg.FollowJointTrajectoryResult.INVALID_JOINTS
            )
            self._error_string = "Unknown joints found: %s" % (str(unknown))
            rospy.logwarn(self._error_string)

    def _moveTo(self, jointNames, point):
        """
        Moves all joints to the positions defined in point

        :param jointNames: Names of joints
        :type jointNames: list
        :param point: Target point
        :type point: trajectory_msgs.msg.JointTrajectoryPoint
        """
        for i in range(0, len(jointNames)):
            message = nicomsg.msg.sff()
            message.param1 = jointNames[i]
            message.param2 = moveitWrapper.rosToNicoAngle(
                jointNames[i], point.positions[i], self.jsonConfig, self.vrep
            )
            message.param3 = self.fractionMaxSpeed
            self._setAngle.publish(message)
        rospy.sleep(0.1)

    def _sendFeedback(self, jointNames, point, start_time):
        """
        Sends feedback about the current status to the action client

        :param jointNames: Names of all joints
        :type jointNames: list
        :param point: Current point
        :type point: trajectory_msgs.msg.JointTrajectoryPoint
        :param start_time: Start time of the trajectory
        :type start_time: rospy.Time
        """
        desired = trajectory_msgs.msg.JointTrajectoryPoint()
        actual = trajectory_msgs.msg.JointTrajectoryPoint()
        error = trajectory_msgs.msg.JointTrajectoryPoint()
        for i in range(0, len(jointNames)):
            joint = jointNames[i]
            desired.positions += [point.positions[i]]
            value = self._getAngle(joint).value
            value = moveitWrapper.nicoToRosAngle(
                joint, value, self.jsonConfig, self.vrep
            )
            actual.positions += [value]
            calculated_error = abs(desired.positions[i] - actual.positions[i])
            error.positions += [calculated_error]
            # Check margin
            if self.config["check_correct_path_execution"]:
                if (
                    joint in self._path_tolerance
                    and calculated_error > self._path_tolerance[joint]
                ) or calculated_error > self.pathTolerance:
                    self._error_code = (
                        control_msgs.msg.FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    )
                    self._error_string = (
                        "Joint %s missed path position by %f (should: %f, is: %f)"
                        % (
                            joint,
                            calculated_error,
                            desired.positions[i],
                            actual.positions[i],
                        )
                    )
                    rospy.logwarn(self._error_string)
                    return

        desired.time_from_start = point.time_from_start
        actual.time_from_start = rospy.Time.now() - start_time
        message = control_msgs.msg.FollowJointTrajectoryFeedback()
        message.desired = desired
        message.actual = actual
        message.error = error
        self._actionServer.publish_feedback(message)

    def _moveToCurrentJointAngles(self, jointNames):
        """
        Sets joint angles to current values to stop any further movement.

        :param jointNames: Names of joints
        :type jointNames: list
        """
        for i in range(0, len(jointNames)):
            message = nicomsg.msg.sff()
            message.param1 = jointNames[i]
            message.param2 = self._getAngle(jointNames[i]).value
            message.param3 = self.fractionMaxSpeed
            self._setAngle.publish(message)

    def _checkFinalPosition(self, followJointMessage, start_time):
        """
        Checks if the final joint positions are in the correct error margin and
        if the goal time is within the time frame

        :param followJointMessage: Trajectory message
        :type followJointMessage: control_msgs.msg.FollowJointTrajectoryAction
        :param start_time: Start time of the trajectory
        :type start_time: rospy.Time
        """
        if (
            self._error_code
            is not control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
        ):
            return
        tolerance_dict = dict()
        for JointTolerance in followJointMessage.goal_tolerance:
            if JointTolerance.position is not -1 and JointTolerance.position is not 0:
                tolerance_dict[JointTolerance.name] = JointTolerance.position

        # Check joint tolerance
        for i in range(0, len(followJointMessage.trajectory.joint_names)):
            joint = followJointMessage.trajectory.joint_names[i]
            if joint in tolerance_dict:
                desired = followJointMessage.trajectory.points[-1].positions[i]
                value = self._getAngle(joint).value
                actual = moveitWrapper.nicoToRosAngle(
                    joint, value, self.jsonConfig, self.vrep
                )
                error = abs(desired - actual)
                if error > tolerance_dict[joint]:
                    self._error_code = (
                        control_msgs.msg.FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                    )
                    self._error_string = (
                        "Joint %s missed goal position by %f (should: %f, is: %f)"
                        % (joint, error, desired, actual)
                    )
                    rospy.logwarn(self._error_string)
                    return

        # Check time tolerance
        time_error = (
            start_time + followJointMessage.trajectory.points[-1].time_from_start
        ) - rospy.Time.now()
        # if no time tolerance is set no error is produced
        if (
            abs(followJointMessage.goal_time_tolerance.secs)
            + abs(followJointMessage.goal_time_tolerance.nsecs)
            > 0.0
        ):
            if (abs(time_error.secs) > followJointMessage.goal_time_tolerance.secs) or (
                abs(time_error.secs) == followJointMessage.goal_time_tolerance.secs
                and abs(time_error.nsecs) > followJointMessage.goal_time_tolerance.nsecs
            ):
                self._error_code = (
                    control_msgs.msg.FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                )
                self._error_string = (
                    "Goal time tolerance violated (tolerance: %s, error: %s)"
                    % (str(followJointMessage.goal_time_tolerance), str(time_error))
                )
                rospy.logwarn(self._error_string)

    def _sendResult(self):
        """
        Sends the final result of the trajectory action to the client
        """
        message = control_msgs.msg.FollowJointTrajectoryResult()
        message.error_code = self._error_code
        message.error_string = self._error_string
        if (
            self._error_code
            is not control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL
        ):
            self._actionServer.set_aborted(message)
        else:
            self._actionServer.set_succeeded(message)


if __name__ == "__main__":
    config = TrajectoryServer.getConfig()

    parser = argparse.ArgumentParser("NICO trajectory server")
    parser.add_argument(
        "--rostopic-name",
        dest="rostopicName",
        help="Topic name for ROS. Default: %s" % config["rostopicName"],
        type=str,
    )
    parser.add_argument(
        "--rosnico-prefix",
        dest="rosnicoPrefix",
        help="Prefix of the NICO ROS motion interface. Default: %s"
        % config["rosnicoPrefix"],
        type=str,
    )
    parser.add_argument(
        "-c",
        "--check",
        dest="check",
        help="Check if joint path is correctly executed",
        action="store_false",
    )

    args = parser.parse_known_args()[0]

    if args.rostopicName:
        config["rostopicName"] = args.rostopicName
    if args.rosnicoPrefix:
        config["rosnicoPrefix"] = args.rosnicoPrefix

    config["check_correct_path_execution"] = args.check

    # server = TrajectoryServer(config)

    leftArm_controller = TrajectoryServer("/leftArm_controller", config)
    rightArm_controller = TrajectoryServer("/rightArm_controller", config)
    leftLeg_controller = TrajectoryServer("/leftLeg_controller", config)
    rightLeg_controller = TrajectoryServer("/rightLeg_controller", config)

    rospy.spin()
