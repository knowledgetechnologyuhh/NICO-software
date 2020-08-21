import json
from os.path import abspath, dirname

from nicomotion.Motion import Motion

parent_dir = dirname(dirname(dirname(dirname(abspath(__file__)))))
config = "{}/json/nico_humanoid_vrep.json".format(parent_dir)
scene = "{}/v-rep/NICO-seated.ttt".format(parent_dir)

# robot = pyrep.from_vrep(
#     config,
#     "{}/NICO-software/v-rep/NICO-seated.ttt".format(parent_dir),
#     headless=True, tracked_objects=["Cuboid"])

vrep_config = Motion.pyrepConfig()
vrep_config["vrep_scene"] = scene
vrep_config["headless"] = False
robot = Motion(config, vrep=True, vrepConfig=vrep_config)

print("Start simulation")

robot.startSimulation()

# print("Cuboid at {}".format(robot.Cuboid.position))

print("Setting goal positions")

# robot.head_z.goal_position = -90
# robot.r_shoulder_y.goal_position = 90
robot.setAngle("head_z", -90, 0.05)
robot.setAngle("r_shoulder_y", 90, 0.05)

print("Stepping...")

for i in range(50):
    robot.nextSimulationStep()
    if i % 5 == 0:
        print("head_z: {}".format(robot.getAngle("head_z")))
        print("r_shoulder_y: {}".format(robot.getAngle("r_shoulder_y")))
print("Stopping")

robot.stopSimulation()

robot.cleanup()
