import time
import pypot.robot

from pypot.primitive.move import MoveRecorder, Move, MovePlayer

from pypot.robot import from_json

#my_robot = from_json('nicu_humanoid.json')
my_robot = from_json('nicu_humanoid_only_upper.json')
#my_robot = from_json('my_robot.json')

my_robot.compliant = True
raw_input("Press Enter to continue...")

