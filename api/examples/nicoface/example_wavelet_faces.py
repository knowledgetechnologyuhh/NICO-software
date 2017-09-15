# Author: Erik
#
# Example for generating different face expression with wavelets
#
from nicoface.FaceExpression import faceExpression
import time

#Change your interface here
fe = faceExpression("/dev/ttyACM0")

#Generate the mouth form and eyebrowse
# Using the standard values
fe.mouth = fe.gen_mouth()
fe.left = fe.gen_eyebrowse()
fe.right = fe.gen_eyebrowse()
fe.send()

time.sleep(1.5)

# Mouth Generator contain 1 tuple for each mouth wavelet
# (ystretch,yoffset,xstretch,xoffset)
fe.mouth = fe.gen_mouth((-0.2,-0.2,1.0,0),(None,None,None,None))

# This can be displayed on the screen
#fe.show_PIL(fe.mouth)

# And send to the NICO
fe.send()

time.sleep(1.5)

#Using both tuples for two mouth lines
fe.mouth = fe.gen_mouth((-1.0,0.2,1.0,0),(0.4,0.1,1.0,0))

fe.send()

time.sleep(1.5)

#Using both tuples for two mouth lines
fe.mouth = fe.gen_mouth((1.0,-0.8,1.0,0),(None,None,None,None))
fe.left = fe.gen_eyebrowse((-1.2,0.2,1.0,0),type="l")
fe.right = fe.gen_eyebrowse(type="r")
fe.show_PIL(fe.left)

#And using the eyebrowse


fe.send()

time.sleep(1.5)
