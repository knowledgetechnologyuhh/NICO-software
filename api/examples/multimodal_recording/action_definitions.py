import sys

# definition of the actions
# name of the action : [wait time for reaching the position]
# the number of wait times defines as well the position steps to go
# the position steps are defined in files named pos_[actionname]_[no of step].csv


actions = {
    "pull": [1.5, 2, 4, 2, 1.5],
    "push": [1.0, 1.0, 1.0, 1.0]
}

def move_action(action):

    for n, wait_duration in enumerate(actions[action]):
        #print "breaks: " + str(wait_duration)
        mov.move_file_position(mover_path + "pos_"+action+"_"+str(n+1)+".csv",
                               subsetfname=mover_path + "subset_right_arm.csv", move_speed=0.05)
        sleep(wait_duration)

    mov.move_file_position(mover_path + "pos_"+action+"_"+str(1)+".csv",
                           subsetfname=mover_path + "subset_right_arm.csv", move_speed=0.05, )


if __name__ == "__main__":

    if len(sys.argv)!=2:
        print ("\nI play the actions on the robot")
        print ("\nPlease start me with one of the defined actions: " + str(actions.keys))
    else:
        # Instructions for the experimenter. Brig the robot in Initial position
        print "\n Please put the robot in position. Right arm on the table. Left arm hanging down. Give RETURN after finished.\n"
        raw_input()

        # Optoforce_sensor
        optoforce_sensor = optoforce(ser_number=None, cache_frequency=40)

        # Put the left arm in defined position
        robot = Motion.Motion(
        "../../../json/nico_humanoid_legged_minimal_for_multimodal_recordings.json", vrep=False)
        mover_path = "../../../moves_and_positions/"
        mov = Mover.Mover(robot, stiff_off=False)
        robot_config = robot.getConfig()

        sleep(2)

        # set the robot to be compliant
        robot.disableTorqueAll()

        robot.openHand('RHand', fractionMaxSpeed=fMS_hand)

        robot.enableForceControl("r_wrist_z", 50)
        robot.enableForceControl("r_wrist_x", 50)

        #robot.enableForceControl("r_indexfingers_x", 50)
        #robot.enableForceControl("r_thumb_x", 50)

        # enable torque of left arm joints
        robot.enableForceControl("head_z", 20)
        robot.enableForceControl("head_y", 20)

        robot.enableForceControl("r_shoulder_z", 20)
        robot.enableForceControl("r_shoulder_y", 20)
        robot.enableForceControl("r_arm_x", 20)
        robot.enableForceControl("r_elbow_y", 20)

        move_action(sys.argv[1])
