#!/usr/bin/python

"""
This script is used to generate an URDF from the original URDF that can be used by MoveIt!

For MoveIt! to properly work it needs a fixed frame, therefore a 'world link'
connecting the world with the torso is put into the original urdf.
MoveIt! has problems working with collision geometry meshes with a high number of vertices
thats why simplified meshes are used.
I use these simplified meshes for visual geometry aswell to reduce computation time
"""

import re

original = open("../../../urdf/complete.urdf", 'r')
moveiturdf = open("moveiturdf/urdf/complete.urdf", 'w')

elbow = False
finger = False

for line in original:
  if '<robot name="complete">' in line:
    moveiturdf.write(line)
    moveiturdf.write('	<link name="world"/>\n\n')
    moveiturdf.write('	<joint name="fixed" type="fixed">\n')
    moveiturdf.write('		<parent link="world"/>\n')
    moveiturdf.write('		<child link="torso:11"/>\n')
    moveiturdf.write('	</joint>\n\n')
  #elif 'elbow_y' in line:
  #  elbow = True
  #  moveiturdf.write(line)
  #elif elbow:
  #  if '<limit' in line:
  #    regex = re.match(r"(.*//)NICO-models/meshes/(.*)", line)
  #    regex = re.match(r"(.*lower=)(.*)( upper=.*)", line)
  #    moveiturdf.write(regex.group(1)+'"0"'+regex.group(3)+'\n')
  #    elbow = False
  #  else:
  #    moveiturdf.write(line)
  #elif '<link name="finger_segment' in line or '<link name="fingertip' in line:
  #  finger = True
  #elif finger:
  #  if '</joint>' in line:
  #    finger = False
  elif 'package://NICO-models/meshes' in line:
    mesh = re.match(r"(.*//)NICO-models/meshes/(.*)", line)
    if 'finger_segment_ipt_50d2a7f4' in line or 'fingertip_ipt_8417b8a9' in line or 'left_upper_arm_iam_49d50008' in line or 'left_wrist_iam_c75e4b6f' in line or 'right_upper_arm_iam_e3e9c979' in line or 'right_wrist_iam_81504746' in line: # this can be removed once ROS indigo is not used anymore
      moveiturdf.write(mesh.group(1)+'moveitmeshes/meshes/'+mesh.group(2)+'\n')
    else:
      moveiturdf.write(mesh.group(1)+'nicoros/../../../urdf/meshes/'+mesh.group(2)+'\n')
  else:
    moveiturdf.write(line)
