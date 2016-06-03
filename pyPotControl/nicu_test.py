import pypot.robot
from pypot.robot import from_json

def move_joint(joint, speed, position):
    print 'Initial position: '+ str(eval('nicu.'+joint+'.present_position'))
    setattr(eval('nicu.'+joint), 'compliant', False)
    setattr(eval('nicu.'+joint), 'goal_speed', speed)
    setattr(eval('nicu.'+joint), 'goal_position', position)

nicu = from_json('nicu_humanoid_only_upper.json')

joint = 'head_z'
move_joint(joint, 8, 90)

raw_input("Press enter to continue...")

print nicu.head_z.present_position


