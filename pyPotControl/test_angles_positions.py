import sys
import time
import pypot.robot
from pypot.robot import from_json

nicu = from_json('nicu_humanoid_only_upper.json')

if len(sys.argv)!= 3:
    print 'USE: python test_angles_positions angleToTest frequencyInSec'
    raw_input('Press ENTER to continue...')
    sys.exit(0)

motor_pos = 'nicu.'+sys.argv[1]+'.present_position'
time_step = float(sys.argv[2])

print 'Showing position of motor '+sys.argv[1]+' every '+sys.argv[2]+' seconds...'

while True:
    print eval(motor_pos)
    time.sleep(time_step)

raw_input("Press enter to continue...")
