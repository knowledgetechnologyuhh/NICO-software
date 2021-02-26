import unittest
import os
from os.path import dirname, abspath, isfile
from nicomotion.Motion import Motion


class PyrepTest(unittest.TestCase):
    def setUp(self):
        assert "COPPELIASIM_ROOT" in os.environ
        assert isfile("{}/coppeliaSim.sh".format(os.environ["COPPELIASIM_ROOT"]))
        assert os.environ["COPPELIASIM_ROOT"] in os.environ["LD_LIBRARY_PATH"]

        parent_dir = dirname(dirname(dirname(dirname(dirname(abspath(__file__))))))
        config = "{}/json/nico_humanoid_vrep.json".format(parent_dir)
        scene = "{}/v-rep/NICO-seated.ttt".format(parent_dir)
        vrep_config = Motion.pyrepConfig()
        vrep_config["vrep_scene"] = scene
        vrep_config["headless"] = True
        self.robot = Motion(config, vrep=True, vrepConfig=vrep_config)

    def tearDown(self):
        del self.robot

    def test_start_stop(self):
        pyrep = self.robot._robot._controllers[0].io.pyrep
        self.robot.startSimulation()
        self.assertTrue(pyrep.running)
        self.robot.stopSimulation()
        self.assertFalse(pyrep.running)

    def test_set_angle(self):
        self.robot.startSimulation()
        self.robot.setAngle("head_z", -90, 0.05)
        for i in range(40):
            self.robot.nextSimulationStep()
        self.assertEqual(-90, self.robot.getAngle("head_z"))
        self.robot.stopSimulation()

    def test_change_angle(self):
        self.robot.startSimulation()
        start = self.robot.getAngle("head_z")
        self.robot.changeAngle("head_z", 45, 0.05)
        for i in range(20):
            self.robot.nextSimulationStep()
        self.assertEqual(start + 45, self.robot.getAngle("head_z"))
        self.robot.stopSimulation()


if __name__ == "__main__":
    unittest.main()
