#!/usr/bin/python

## Used to switch between different joint constraints

#USAGE: Use script with a certain setting parameter (e.g. elbow_free, elbow_constraint)
# python setJointConstraints.py elbow_free
# python setJointConstraints.py elbow_constraint

import sys
import fileinput

constraint = sys.argv[1]
  
if constraint == 'elbow_free':
  elbow_min = -3.14159
  elbow_max = 3.14159
elif constraint == 'elbow_constraint':
  elbow_min = 0.0
  elbow_max = 1.745
elif constraint == 'elbow_very_constraint':
  elbow_min = 0.0
  elbow_max = 0.1  

if constraint == 'elbow_free' or constraint == 'elbow_constraint':
  
  config_file = open("moveiturdf/urdf/complete.urdf", 'r')
  lines = []
  for line in config_file:
    lines.append(line)  
  config_file.close()

  elbow = False
  config_file = open("moveiturdf/urdf/complete.urdf", 'w')
  for line in lines:
    if "<joint name=\"l_elbow_y\"" in line:
      elbow = True
    if "</joint>" in line:
      elbow = False
    if elbow and "<limit" in line:
      config_file.write("		<limit effort=\"30\" lower=\""+ str(elbow_min) +"\" upper=\""+ str(elbow_max) +"\" velocity=\"1\"/>" + "\n")
    else:
      config_file.write(line)
  config_file.close()
  
  print "Elbow joint constraints set to: max_position: " + str(elbow_max) + " , min_position: " + str(elbow_min)


  #config_file = open("moveitgenerated/config/joint_limits.yaml", 'r')
  #lines = []
  #for line in config_file:
  #  lines.append(line)  
  #config_file.close()

  #elbow = False
  #config_file = open("moveitgenerated/config/joint_limits.yaml", 'w')
  #for line in lines:
  #  if "l_elbow_y" in line:
  #    elbow = True
  #  if "l_hip_x" in line:
  #    elbow = False
  #  if elbow and "max_position" in line:
  #    config_file.write("    max_position: " + str(elbow_max) + "\n")
  #  elif elbow and "min_position" in line:
  #    config_file.write("    min_position: " + str(elbow_min) + "\n")
  #  else:
  #    config_file.write(line)
  #config_file.close()

  


