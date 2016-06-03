import pypot.robot
from pypot.robot import from_json

def move_joint(joint, speed, position):
    #print 'Initial position: '+ str(eval('nicu.'+joint+'.present_position'))
    setattr(eval('nicu.'+joint), 'compliant', False)
    setattr(eval('nicu.'+joint), 'goal_speed', speed)
    setattr(eval('nicu.'+joint), 'goal_position', position)

def go(location, speed):
    move_joint('r_shoulder_z', speed, location[0])
    move_joint('r_shoulder_y', speed, location[1])
    move_joint('r_arm_x', speed, location[2])
    move_joint('r_elbow_y', speed, location[3])
    move_joint('r_wrist_z', speed, location[4])

def get(object):
    #it should depend of the object, for the just an example
    if object == 'cup':
        move_joint('r_gripper_x', 5, -20)
    if object == 'sponge':
        move_joint('r_gripper_x', 5, 10)

def drop(object):
    move_joint('r_gripper_x', 5, 50)

nicu = from_json('nicu_humanoid_only_upper_tmp.json')

home_pos = [0,0,-5,85,0]
left_pos = [25,25,5.5,60,15]
right_pos = [50,32,5,50,10]

go(right_pos, 5)
drop('cup')


raw_input("Press enter to continue...")


