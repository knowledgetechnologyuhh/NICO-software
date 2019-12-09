from time import sleep
import serial
import sys
from FaceExpression import faceExpression
import logging

def printCapacitiveReadings():	
	
	readings = myFace.getCapacitiveReadings()
	print("{} pads. Values:".format(len(readings))),
	print readings
	
	# if readings need to be recallibrated, call
	# 		myFace.recallibrateCapacitivePads().
	# this should not normally be needed because a callibration is automatically performed
	# when the face controller is powered on.
		

if __name__ == "__main__":
	print "Getting capacitive touch readings from Nico face. Press Ctrl+C to terminate"

	logging.basicConfig() # needed for logger in FaceExpression	
	myFace = faceExpression("COM14", "real")

	while(1):
		printCapacitiveReadings()
