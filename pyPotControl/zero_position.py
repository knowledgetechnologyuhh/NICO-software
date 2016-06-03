import pypot.robot

from pypot.robot import from_json

#my_robot = from_json('nicu_humanoid_only_upper.json')
my_robot = from_json('nicu_humanoid.json')

for m in my_robot.motors:
	m.compliant=False
	m.goal_speed=5
	m.goal_position=0

#This is needed to let the frimework the time to set the joints read
raw_input("Press Enter to continue...")

