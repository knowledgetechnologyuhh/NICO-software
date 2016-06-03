from time import sleep
import pypot.robot
import random

from pypot.primitive.move import MoveRecorder, Move, MovePlayer

from pypot.robot import from_json

#my_robot = from_json('nicu_humanoid_only_upper_with_hands.json')
my_robot = from_json('seed_right_hand.json')
#my_robot = from_json('my_smaller_robot.json')

my_robot.compliant = True

for m in my_robot.motors:
	print m.name + " : " + str(m.id) + " : " +str(m.present_position) + " : limits " + str(m.angle_limit) + " goal_speed " + str(m.goal_speed)

for m in my_robot.motors:

	#m.compliant=False
	m.goal_speed=1
	#m.goal_position=0

my_robot.r_thumb_x.compliant=False
my_robot.r_thumb_x.goal_speed=10
my_robot.r_thumb_x.goal_position=random.randint(10,100)
sleep(2)
my_robot.r_thumb_x.compliant=True
#my_robot.r_thumb_x.compliant=True
#my_robot.l_elbow_y.compliant=False
#my_robot.l_elbow_y.goal_speed=5
#my_robot.l_elbow_y.goal_position=-10
#my_robot.l_elbow_y.compliant=False
#my_robot.l_shoulder_y.goal_speed=5
#my_robot.l_shoulder_y.goal_position=20
#my_robot.l_knee_y.compliant=False
#my_robot.l_knee_y.goal_speed=5
#my_robot.l_knee_y.goal_position=0




#time.sleep(5)
#my_robot.head_z.goal_position=170
#time.sleep(5)
#my_robot.head_z.goal_position=-170


#dxl_io.set_goal_position({21: 2.00})
#dxl_io.set_goal_position({22: -12.00})

#print(dxl_io.get_present_position(ids))

raw_input("Press Enter to continue...")

