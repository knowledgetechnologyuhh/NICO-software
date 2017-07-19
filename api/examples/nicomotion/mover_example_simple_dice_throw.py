#!/usr/bin/env python

# Very simple example to use the mover class
# Toggle the right arm between two position and open and close the hands (Leads to a dice throw, if you put the dice in the robots hand)


from nicomotion import Motion
from nicomotion import Mover
import time


mover_path="../../../moves_and_positions/"
robot=Motion.Motion("../../../json/nico_humanoid_upper.json",vrep=False)
mov = Mover.Mover(robot,stiff_off=True)

key="c"
while key != "q":
    mov.move_file_position(mover_path+"pos_get_dice.csv",subsetfname=mover_path+"subset_right_arm.csv")
    robot.openHand("RHand")
    raw_input()
    robot.closeHand("RHand")
    time.sleep(2)
    robot.openHand("RHand")
    ttw=mov.move_file_position(mover_path+"pos_throw_dice.csv",subsetfname=mover_path+"subset_right_arm.csv",move_speed=0.5)
    print "Waiting for " + str(ttw) + " seconds."
    time.sleep(ttw)
    key=raw_input()