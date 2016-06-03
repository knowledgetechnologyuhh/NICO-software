# -*- coding: utf-8 -*-
"""
Created on Wed Oct 7 16:12:18 2015

@author: cruz
"""

import vrep
import math

class Simulator(object):

    def __init__(self):
        self.clientID = self.connectRobot()
    #end of __init__ method

    def connectRobot(self):
        clientID=vrep.simxStart('127.0.0.1',19997,True,False,5000,5)
        if clientID!=-1:
            print 'Connected to remote API server with clientID: ', clientID
            vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)
        else:
            print 'Failed connecting to remote API server'

        return clientID
    #end of connectRobot method
    
    def disconnectSimulator(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
        vrep.simxFinish(self.clientID)
        print 'Robot and simulator disconnected'
    #end of disconnectSimulator method

    def moveJoint(self, jointToMove, angle):
        errorCode, handleJoint = vrep.simxGetObjectHandle(self.clientID, jointToMove, vrep.simx_opmode_oneshot_wait)
        if errorCode==vrep.simx_error_noerror:
            vrep.simxSetJointTargetPosition(self.clientID, handleJoint, angle, vrep.simx_opmode_oneshot)
        else:
            print 'Error. Got no handle: ', errorCode
    #end of moveJoint method
            
if __name__ == "__main__":
    simulator = Simulator()

    simulator.moveJoint('r_arm_x#', math.pi/2)

    raw_input('Press enter to continue...')
    simulator.disconnectSimulator()
    print 'The end'

#end of class Simulator
