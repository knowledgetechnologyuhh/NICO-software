import time
import pypot.robot

from pypot.primitive.move import MoveRecorder, Move, MovePlayer

from pypot.robot import from_json

my_robot = from_json('nicu_humanoid_only_upper_tmp.json')
#my_robot = from_json('my_smaller_robot.json')

my_robot.compliant = True

for m in my_robot.motors:
	print m.name + " : " + str(m.id) + " : " +str(m.present_position) + " : limits " + str(m.angle_limit) + " goal_speed " + str(m.goal_speed)

for m in my_robot.motors:

	m.compliant=False
	m.goal_speed=5
	m.goal_position=0

#my_robot.r_knee_y.compliant=False
#my_robot.r_knee_y.goal_speed=5
#my_robot.r_knee_y.goal_position=0
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

