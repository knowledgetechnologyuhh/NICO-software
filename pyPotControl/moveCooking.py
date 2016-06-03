
import time
import pypot.robot

from pypot.primitive.move import MoveRecorder, Move, MovePlayer

from pypot.robot import from_json

import logging

#logging.basicConfig(filename='example.log',level=logging.DEBUG)

my_robot = from_json('my_robot_simple.json')

with open('mix2.move') as f:
    m = Move.load(f)

my_robot.compliant = False

move_player = MovePlayer(my_robot, m)
move_player.start()

time.sleep(45)

with open('presentGlass.move') as f:
    m = Move.load(f)

my_robot.compliant = False

move_player = MovePlayer(my_robot, m)
move_player.start()


raw_input("Press Enter to continue...")
