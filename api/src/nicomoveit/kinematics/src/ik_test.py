#!/usr/bin/env python

import ikpy
import numpy as np
from ikpy import chain
import transforms3d
import math
import time

def angleDiff(fwk,target_frame):
    i_al, i_be, i_ga = transforms3d.taitbryan.mat2euler(fwk[:3, :3])
    t_al, t_be, t_ga = transforms3d.taitbryan.mat2euler(target_frame[:3, :3])
    d_al = i_al - t_al
    d_be = i_be - t_be
    d_ga = i_ga - t_ga
    orientation_distance = math.sqrt(d_al * d_al + d_be * d_be + d_ga * d_ga)
    return(abs(d_al), abs(d_be), abs(d_ga), orientation_distance)

class ik_test:
  """
  Class which allows to make tests with python iks
  """
  def __init__(self, method, useGrid = False, experiment_count = 0, kinematics = "", moveiturdf = "", goalsFileName = "goals", reachedPosesProvided = False, providedReachedPosesFileName = "reached"):
    #useGrid = False
    #reachedPosesProvided = False
    
    self.method = method
    
    self.useGrid = useGrid
    self.reachedPosesProvided = reachedPosesProvided
    
    self.kinematics = kinematics

    jointValuesFileName = kinematics+"/results/results_"+str(experiment_count)+"/jointValues"
    if not self.reachedPosesProvided: 
      self.jointValueFile = open(jointValuesFileName,'w')
    
    # load file with test positions and save data in 'goals'
    lines = []
    goalsFile = open(goalsFileName,'r')
    for line in goalsFile:
      lines.append([float(i) for i in line.split()])
    goalsFile.close()

    goals = []
    if self.useGrid:
      goals = lines
    else:
      # if the file contains the whole pose we have to parse multiple lines to one goal
      i = 0
      goal = []
      for line in lines:
        i += 1
        if i < 5:
          goal.append(line)
        else:
          goals.append(goal)
          goal = []
          i = 0

    self.goals = goals

    if self.reachedPosesProvided:
      # load file with provided reached poses and save data in 'providedPoses'
      lines = []
      providedReachedPosesFile = open(providedReachedPosesFileName,'r')
      for line in providedReachedPosesFile:
        if "stats" in line:
          self.stats = line
        else:
          lines.append([float(i) for i in line.split()])
      providedReachedPosesFile.close()

      providedPoses = []
      i = 0
      pose = []
      for line in lines:
        i += 1
        if i < 5:
          pose.append(line)
        else:
          providedPoses.append(pose)
          pose = []
          i = 0

      self.providedPoses = providedPoses

    # build kinematic chain 
    mask = [False,True,True,True,True,True,True]
    self.leftArm = chain.Chain.from_urdf_file(moveiturdf+"/urdf/complete.urdf", base_elements=["torso:11","l_shoulder_z"], active_links_mask=mask)



  def test(self):
    nofRequests = 0; positives = 0; negatives = 0
    elapsedSecs = 0.0; elapsedSecsForPositives = 0.0; timeUsed = 0.0
    distanceToTargets = 0.0; overallXdif = 0.0; overallYdif = 0.0; overallZdif = 0.0; overallDif = 0.0

    #methods = ["L-BFGS-B","SLSQP","ga_simple"]

    reached = None

    for goal in self.goals:
      nofRequests += 1
      if self.useGrid:
        # use a side grasp orientation  
        target = np.array([[-0.0121975, -0.224666,  0.974359,   0.0],      \
                           [-0.0261838, -0.974026, -0.224917,   0.0],      \
                           [0.999583,   -0.0282559, 0.00599805, 0.0],      \
                           [0.0,         0.0,       0.0,        1.0]])  
        target[0][3] = goal[0]
        target[1][3] = goal[1]
        target[2][3] = goal[2]
        positionTarget = goal
        
      else:
        target = np.array(goal)
        positionTarget = target[:3,3]
        
      if not self.reachedPosesProvided:  
        print "Target: "
        print target  
        # compute inverse kinematics and measure time    
        started = time.time()
        if self.method == "L-BFGS-B" or self.method == "SLSQP":
          reached = self.leftArm.inverse_kinematics(target, method = self.method, include_orientation = True)     
        if self.method == "ga_simple":               
          reached = self.leftArm.inverse_kinematics(target, method = self.method, include_orientation = True,numGenerations=180, or_acc=0.01, dist_acc = 0.01)
        timeUsed = time.time() - started
        elapsedSecs += timeUsed
           
        # compute forward kinematics to measure difference to target
        invPose = self.leftArm.forward_kinematics(reached)
        print "Reached: "
        print invPose    
      
      else:
        invPose = np.array(self.providedPoses[nofRequests-1])
        
      euclideanDistance  = np.linalg.norm(invPose[:3, -1] - positionTarget)    
      Xdif, Ydif, Zdif, squared = angleDiff(invPose,target)

      # check if solution is in tolerance margin
      translationTol = 0.01  # +-1cm            #0.05            # +-5 cm
      orientationTol = 0.01  # +- 0.57 degrees  #0.785398        # +-45 degrees
      if euclideanDistance < translationTol and Xdif < orientationTol and Ydif < orientationTol and Zdif < orientationTol:
        print "Success"
        if not self.reachedPosesProvided: 
          self.jointValueFile.write(str(reached[1]))
          for jointValueIdx in range(len(reached)-2):      
            self.jointValueFile.write(','+str(reached[jointValueIdx+2]))
          self.jointValueFile.write('\n')
        positives += 1
        distanceToTargets += euclideanDistance    
        overallXdif += Xdif
        overallYdif += Ydif
        overallZdif += Zdif
        overallDif  += squared
        elapsedSecsForPositives += timeUsed
      else:
        print "Failure"
        negatives += 1      
        
    if not self.reachedPosesProvided: 
      self.jointValueFile.close()

    # jointValues = []
    # jointValues.append(math.radians(0))
    # jointValues.append(math.radians(-33.2419847909844))
    # jointValues.append(math.radians(-173.24035509280196))
    # jointValues.append(math.radians(55.712904379535324))
    # jointValues.append(math.radians(15.921103729229518))
    # jointValues.append(math.radians(-21.114608419745192))
    # jointValues.append(math.radians(-37.51970316823181))
    # self.leftArm.forward_kinematics(jointValues)

    if self.reachedPosesProvided:
      stats = self.stats.split()
      positives = int(stats[1])
      negatives = int(stats[2])
      nofRequests = positives + negatives
      elapsedSecs = float(stats[3])
      elapsedSecsForPositives = float(stats[4])
      

    print "Percentage of successful IK solutions: " + str(positives/nofRequests) + " ("+ str(positives) + " positives and " + str(negatives) + " negatives)"
    print "Overall processing time: " + str(elapsedSecs)
    print "Average processing time per request: " + str(elapsedSecs/nofRequests)
    
    statsFile = open(self.kinematics+"/statistics/experiment", 'a')
    statsFile.write(","+str(positives) + "," + str(negatives) + "," + str(elapsedSecs / (positives + negatives)) + ",")
    
    if positives > 0:    
      print "Average processing time per successful request: " + str(elapsedSecsForPositives/positives)
      print "Average distance to target in successful attempts: " + str(distanceToTargets/positives)
      print "Average squared orientation distance: " + str(overallDif/positives)
      print "Average difference to x axis orientation: " + str(overallXdif/positives)
      print "Average difference to y axis orientation: " + str(overallYdif/positives)
      print "Average difference to z axis orientation: " + str(overallZdif/positives)   
    
      statsFile.write(str(elapsedSecsForPositives / positives) + "," + str(distanceToTargets/positives) + "," + str(overallXdif/positives) + "," + str(overallYdif/positives) + "," + str(overallZdif/positives))
    else:
      statsFile.write(str(-1) + "," + str(-1) + "," + str(-1) + "," + str(-1) + "," + str(-1))
    statsFile.close()


def main(): 
  methods = ["L-BFGS-B","SLSQP","ga_simple"]
  ik_test_instance = ik_test(methods[0])
  ik_test_instance.test()













