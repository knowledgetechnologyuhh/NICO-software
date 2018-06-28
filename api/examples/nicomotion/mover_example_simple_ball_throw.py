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
    #mov.play_movement(mover_path+"mov_from_table_to_get.csv",subsetfname=mover_path+"subset_left_arm_and_head.csv",move_speed=0.01)
    mov.move_file_position(mover_path + "pos_from_table_to_get.csv", subsetfname=mover_path + "subset_left_arm_and_head.csv",
                      move_speed=0.05)
    robot.openHand("LHand")
    raw_input()
    robot.closeHand("LHand",1.0,0.5)
    time.sleep(2)
    #mov.play_movement(mover_path+"mov_from_get_throw.csv",subsetfname=mover_path+"subset_left_arm.csv",move_speed=0.03)
    ttw=mov.move_file_position(mover_path + "pos_from_get_throw.csv", subsetfname=mover_path + "subset_left_arm.csv",
                      move_speed=0.05)
    print ttw
    #time.sleep(ttw)
    time.sleep(4.5)
    robot.openHand("LHand",1.0,0.7)
    time.sleep(0.5)
    ttw=mov.move_file_position(mover_path+"pos_throw_ball.csv",subsetfname=mover_path+"subset_left_arm.csv",move_speed=0.2)
    time.sleep(ttw)
    key=raw_input()