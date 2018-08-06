import sys
from nicomotion import Motion
from time import sleep







if __name__ == "__main__":

        robot = Motion.Motion("../../../json/nico_humanoid_legged_minimal_for_multimodal_recordings.json", vrep=False)
        mover_path = "../../../moves_and_positions/"
        robot_config = robot.getConfig()

        sleep(2)

        # set the robot to be compliant
        #MK: I deactivated this to prevent the robot from "bashing" it's hand into the table between trials
        # The robot hand is always above the table and does not scrape over the surface
        robot.disableTorqueAll()

        #robot.openHand('RHand', fractionMaxSpeed=0.8)

        robot.enableForceControl("r_wrist_z", 5)
        robot.enableForceControl("r_wrist_x", 5)

        robot.enableForceControl("r_indexfingers_x", 50)
        robot.enableForceControl("r_thumb_x", 50)

        # enable torque of left arm joints
        #robot.enableForceControl("head_z", 20)
        #robot.enableForceControl("head_y", 20)

        #robot.enableForceControl("r_shoulder_z", 20)
        #robot.enableForceControl("r_shoulder_y", 20)
        #robot.enableForceControl("r_arm_x", 20)
        #robot.enableForceControl("r_elbow_y", 20)

        while(True):
            #robot.setAngle("r_thumb_x", -160, 0.4)
            #robot.setAngle("r_indexfingers_x", -160, 0.4)
            print("Set to -180")
            robot.setAngle("r_wrist_z", -180, 0.02)
            for i in range(100):
                print("ANGLE= " + str(robot.getAngle("r_wrist_z")))
                sleep(0.1)

            print("Set to 180")
            robot.setAngle("r_wrist_z", 180, 0.02)
            for i in range(100):
                print("ANGLE= " + str(robot.getAngle("r_wrist_z")))
                sleep(0.1)


