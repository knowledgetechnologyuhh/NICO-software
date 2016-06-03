import time
import pypot.robot
import sys

from pypot.primitive.move import MoveRecorder, Move, MovePlayer

from pypot.robot import from_json

def setMotor(motorID,angle):

	my_robot = from_json('nicu_humanoid.json')
#my_robot = from_json('my_robot.json')

	for m in my_robot.motors:
		print m.name + " : " + str(m.id) + " : " +str(m.present_position) + " : limits " + str(m.angle_limit) + " goal_speed " + str(m.goal_speed)
		if (m.id==motorID):
			print "Setting: " + str(motorID)
			m.compliant=False
			m.goal_speed=5
			m.goal_position=angle


if __name__ == "__main__":
   if (len(sys.argv)!=3):
	print "setMotor.py MotorID angle"
   else:   
     setMotor(int(sys.argv[1]),float(sys.argv[2]))
     raw_input("Press Enter to continue...")

