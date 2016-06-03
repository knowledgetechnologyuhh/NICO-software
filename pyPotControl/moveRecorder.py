import time
import pypot.robot

from pypot.primitive.move import MoveRecorder, Move, MovePlayer

from pypot.robot import from_json

#my_robot = from_json('my_robot.json')
my_robot = from_json('nicu_humanoid_only_upper_tmp.json')


move_recorder = MoveRecorder(my_robot, 100, my_robot.motors)

print ("Move recorder. First I have to relax the joints.Please wait a little bit.")
my_robot.compliant = True
time.sleep(2)
print("Press Enter to start...")
raw_input()
print ("Start the movement in 1 second.")
time.sleep(1)
print ("Start the movement now.")
move_recorder.start()

#time for recording
time.sleep(30)
move_recorder.stop()
print ("Recording over.")

filename = 'headMovement.move'
with open(filename, 'w') as f:
    move_recorder.move.save(f)
    print ("Saving.")
    time.sleep(1)
