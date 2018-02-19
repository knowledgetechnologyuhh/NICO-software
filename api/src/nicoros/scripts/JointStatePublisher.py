#!/usr/bin/env python

import argparse
import logging
import threading
import math
import time
import ast

import rospy
import nicomsg.msg
import nicomsg.srv
import sensor_msgs.msg

from nicomoveit import moveitWrapper

class JointStatePublisher():
    """
    The JointStatePublisher continuously publishes the current joint states of NICO.
    """

    @staticmethod
    def getConfig():
        """
        Returns a default config dict

        :return: dict
        """
        return {'rosnicoPrefix': '/nico/motion',
                'jointStateName': '/joint_states',
                'sittingPosition': True
                }


    def __init__(self, config = None):
        """
        The JointStatePublisher provides the current joint states via ROS

        :param config: Configuration
        :type config: dict
        """
        if config is None:
          config = TrajectoryServer.getConfig()
        self._running = True
        self.config = config
        self.count = 0
        
        logging.info('-- Init JointStatePublisher --')
        
        logging.debug('Init ROS')
        rospy.init_node('nicojointstate', anonymous=True)
      
        logging.info('Waiting for rosnico')
        rospy.wait_for_service('%s/getAngle' % config['rosnicoPrefix'])
        
        if rospy.has_param(config['rosnicoPrefix']+'/sittingPosition'):
            config['sittingPosition'] = rospy.get_param(config['rosnicoPrefix']+'/sittingPosition')

        logging.debug('Init service proxies')
        self._getAngle = rospy.ServiceProxy('%s/getAngle' % config['rosnicoPrefix'], nicomsg.srv.GetValue )
        self._getJointNames = rospy.ServiceProxy('%s/getJointNames' % config['rosnicoPrefix'], nicomsg.srv.GetNames )
        self._getConfig = rospy.ServiceProxy('%s/getConfig' % config['rosnicoPrefix'], nicomsg.srv.GetString )
        self.jsonConfig = ast.literal_eval(self._getConfig().data)
        self._getVrep = rospy.ServiceProxy('%s/getVrep' % config['rosnicoPrefix'], nicomsg.srv.GetString )        
        self.vrep = bool(self._getVrep().data)
        
        logging.debug('Init publishers')
        self._jointStatePublisher = rospy.Publisher('%s' % config['jointStateName'], sensor_msgs.msg.JointState, queue_size = 1)
        self._jointStateThread = threading.Thread(target=self._sendJointState)
        self._jointStateThread.start()        
        
        logging.info('-- All done --')
      
    def stop(self):
        self._running = False
        self._jointStateThread.join()
      
    def _sendJointState(self):
        """
        Loop for sending the current joint state
        """
        while self._running:
            self.count += 1
            if self.count > 9:
                self.count = 0
                if rospy.has_param(self.config['rosnicoPrefix']+'/sittingPosition'):
                    self.config['sittingPosition'] = rospy.get_param(self.config['rosnicoPrefix']+'/sittingPosition')
            message = sensor_msgs.msg.JointState()
            message.name = []
            message.position = []
            message.effort = []
            #joints = self.robot.getJointNames()
            joints = self._getJointNames().names
            
            for joint in joints:
                message.name += [joint]
                #value = self.robot.getAngle(joint)
                value = self._getAngle(joint).value
                value = moveitWrapper.nicoToRosAngle(joint, value, self.jsonConfig, self.vrep)
                rospy.loginfo(joint+' '+str(value))
                message.position += [value]
                #message.effort += [self.robot.getLoad(joint)]
            
            joints_without_motor = []
            leftLeg = ['l_hip_z', 'l_hip_x', 'l_hip_y', 'l_knee_y', 'l_ankle_y', 'l_ankle_x']
            leftLeg_sitting = [0.0601951067222, -0.084078543203, -1.53735464204, 1.24698948964, 0.305836083766, 0.0223105563298]
            rightLeg = [ 'r_hip_z', 'r_hip_x', 'r_hip_y', 'r_knee_y', 'r_ankle_y', 'r_ankle_x']
            rightLeg_sitting = [-0.031318849578, 0.0614412972261, -1.50805089035, 1.20882593629, 0.387770525688, -0.0962579440558]
            if self.config['sittingPosition']:
                for i in range(len(leftLeg)):
                    message.name += [leftLeg[i]]
                    value = leftLeg_sitting[i]
                    message.position += [value]
                    message.name += [rightLeg[i]]
                    value = rightLeg_sitting[i]
                    message.position += [value]                
            else:
                joints_without_motor.extend(leftLeg)
                joints_without_motor.extend(rightLeg)
            
            joints_without_motor.extend(['l_indexfinger_1st_x', 'l_indexfinger_2nd_x', 'l_ringfingers_x', 'l_ringfinger_1st_x', 'l_ringfinger_2nd_x', 'l_thumb_1st_x', 'l_thumb_2nd_x', 'r_indexfinger_1st_x', 'r_indexfinger_2nd_x', 'r_ringfingers_x', 'r_ringfinger_1st_x', 'r_ringfinger_2nd_x', 'r_thumb_1st_x', 'r_thumb_2nd_x'])
            for joint in joints_without_motor:
                message.name += [joint]
                value = 0.0
                message.position += [value]
                
            message.header.stamp = rospy.get_rostime()
            self._jointStatePublisher.publish(message)
            time.sleep(0.25)        
        
if __name__ == '__main__':
    config = JointStatePublisher.getConfig()

    parser = argparse.ArgumentParser('NICO jointstate server')
    parser.add_argument('--rosnico-prefix', dest='rosnicoPrefix', help='Prefix of the NICO ROS motion interface. Default: %s' % config['rosnicoPrefix'], type=str)
    parser.add_argument('-j', '--joint-state-name', dest='jointStateName', help='Name of the joint state topic. PLEASE NOTE: A lot of other nodes assume the joint state note to be at /joint_states, so be careful when changing it. Default: %s' % config['jointStateName'], type=str)

    args = parser.parse_known_args()[0]

    if args.rosnicoPrefix:
        config['rosnicoPrefix'] = args.rosnicoPrefix
    if args.jointStateName:
        config['jointStateName'] = args.jointStateName

    server = JointStatePublisher(config)
    rospy.spin()






