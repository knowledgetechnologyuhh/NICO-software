#!/usr/bin/env python

import time
import math
from nicomoveit import moveitWrapper
from nicomotion import Motion

class groupHandle:
  """
  The groupHandle class provides benchmarking functionalities.
  For a motion planning group a set of possible poses can be generated
  and results can be reported to receive statistics
  """
  def __init__(self, groupName):
    """
    :param groupName: Name of the planning group that should be moved.
                      Possible movement groups to use are: 
                      'leftArm', 'rightArm', 'leftLeg', 'rightLeg'
    :type groupName: str
    """
    self.handle = moveitWrapper.groupHandle(groupName) # always neccessary when using this class?
    self.listOfPoses = []

  def createRandomReachablePoses(self, fileName = None, N = 1, collisionAllowed = False):
    """
    Function to generate a file with N reachable joint configurations.
    One joint configuration per line.
    
    :param N: Number of reachable joint configurations to generate
    :type N: integer
    :param fileName: Output file name (optional)
    :type fileName: str
    :return: list of random reachable poses
    :rtype: list of lists of joint values (floats)
    """
    jointNames = ["l_shoulder_z","l_shoulder_y","l_arm_x","l_elbow_y","l_wrist_z","l_wrist_x"]
    constraints_min = [-2.182,-3.124,-1.8675,0.0,-1.571,-0.872665]
    constraints_max = [1.745,3.124,1.309,1.74533,1.571,0.0]

    if fileName is not None:
      # create a new output file
      outputFile = open(fileName, 'w')
    numberOfPoses = 0
    while numberOfPoses < N:
      target = self.handle.getRandomJointValues()
      if collisionAllowed:
        collision = False
      else:
        collision = self.handle.isColliding(target)        
      if collision:
        print "Collision"
      else:
        outOfBounds = False
        for jointIdx in range(len(target)):
          target[jointIdx] = math.radians(target[jointIdx])
          if target[jointIdx] > constraints_max[jointIdx] or target[jointIdx] < constraints_min[jointIdx]:
            print "Joint " + jointNames[jointIdx] + " value " + str(target[jointIdx])
            outOfBounds = True
        if outOfBounds:
          print "Out of bounds"
        else:
          numberOfPoses = numberOfPoses + 1
          print numberOfPoses
          self.listOfPoses.append(target)
          if fileName is not None:
            line = str(target)[1:-1]
            line = line.replace(',', '')
            outputFile.write(str(line) + '\n')
    if fileName is not None:   
      outputFile.close()
    #return self.listOfPoses
  
  def createGrid(self, fileName, xFrom, xStep, xNofSteps, yFrom, yStep, yNofSteps, zFrom, zStep = 0.1, zNofSteps = 0):
    """
    Function to generate a grid in cartesian space that might be used
    as a testing grid for the inverse kinematics.
    :param fileName: Output file name
    :type fileName: str
    :param xFrom: Starting position on x axis
    :type xFrom: float
    :param xStep: Stepsize on x axis
    :type xStep: float
    :param xNofSteps: Number of steps into the x direction
    :type xNofSteps: integer
    :param yFrom: Starting position on y axis
    :type yFrom: float
    :param yStep: Stepsize on y axis
    :type yStep: float
    :param yNofSteps: Number of steps into the y direction
    :type yNofSteps: integer
    :param zFrom: Starting position on z axis
    :type zFrom: float
    :param zStep: Stepsize on z axis
    :type zStep: float
    :param zNofSteps: Number of steps into the z direction
    :type zNofSteps: integer
    """
    outputFile = open(fileName, 'w')
    for idx_x in range(xNofSteps):    
      p_x = xFrom + idx_x * xStep
      for idx_y in range(yNofSteps):
        p_y = yFrom + idx_y * yStep
        for idx_z in range(zNofSteps):    
          p_z = zFrom + idx_z * zStep
          outputFile.write(str(p_x) + ", " + str(p_y) + ", " + str(p_z) + '\n')
    outputFile.close()
          
  def moveitRandomTest(self, fileName):
    """
    Function to test for all poses in file if a path can be found.
    This function is not intended to be used to test different IKs.
    An appropriate IK testing function is written in c++.
    :param fileName: Output file name
    :type fileName: str
    """
    inputFile = open(fileName,'r')
    poses = []
    for line in inputFile:
      poses.append([float(i) for i in line.split()])
    inputFile.close()
    for pose in poses:
      self.handle.moveToJointCoordinates(pose)
      planningTime = self.handle.planningTime
      executionTime = self.handle.executionTime
    














