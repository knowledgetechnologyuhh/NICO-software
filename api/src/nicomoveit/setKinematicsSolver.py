#!/usr/bin/python

## Used to switch between KDL and trac_ik as kinematic solver

#USAGE: Use script with 'KDL' or 'trac_ik' as parameter
# python setKinematicsSolver.py KDL
# python setKinematicsSolver.py trac_ik

import sys
import fileinput

solver = sys.argv[1]

if solver == 'trac_ik' or solver == 'TRAC_IK' or solver == 'trac' or solver == 'TRAC':
  for line in fileinput.input("moveitgenerated/config/kinematics.yaml", inplace = True): 
    sys.stdout.write(line.replace("kdl_kinematics_plugin/KDLKinematicsPlugin", "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin"))
  for line in fileinput.input("moveitgenerated/config/kinematics.yaml", inplace = True): 
    sys.stdout.write(line.replace("bio_ik/BioIKKinematicsPlugin", "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin"))
elif solver == 'KDL' or solver == 'kdl':
  for line in fileinput.input("moveitgenerated/config/kinematics.yaml", inplace = True): 
    sys.stdout.write(line.replace("trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin", "kdl_kinematics_plugin/KDLKinematicsPlugin"))
  for line in fileinput.input("moveitgenerated/config/kinematics.yaml", inplace = True): 
    sys.stdout.write(line.replace("bio_ik/BioIKKinematicsPlugin", "kdl_kinematics_plugin/KDLKinematicsPlugin"))
elif solver == 'bio_ik' or solver == 'BIO_IK' or solver == 'bio' or solver == 'BIO':
  for line in fileinput.input("moveitgenerated/config/kinematics.yaml", inplace = True): 
    sys.stdout.write(line.replace("trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin", "bio_ik/BioIKKinematicsPlugin"))
  for line in fileinput.input("moveitgenerated/config/kinematics.yaml", inplace = True): 
    sys.stdout.write(line.replace("kdl_kinematics_plugin/KDLKinematicsPlugin", "bio_ik/BioIKKinematicsPlugin"))
else:
  print "Please use either KDL, trac_ik or bio_ik as kinematic solvers"
  


