import gc
import math
import unittest

from ikpy.utils import geometry
from nicomotion import Visualizer
from numpy.testing import assert_array_equal


class VisualizerTest(unittest.TestCase):
    def setUp(self):
        self.visualizer = Visualizer.Visualizer()

    def tearDown(self):
        # ensure visualizer is removed and garbage collected between tests
        del self.visualizer
        gc.collect()

    def test_set_angle(self):
        """Check if angle is set correctly"""
        # set angle
        joint = "r_shoulder_z"
        angle = 45.0
        self.visualizer.set_angle(joint, angle)
        # check if angle in the robot's config matches
        self.assertEqual(self.visualizer.robot.cfg, {joint: math.radians(angle)})

    def test_set_angles(self):
        """Checks if setting multiple angles works as intended"""
        # set two angles at once
        joints = ["r_shoulder_y", "r_shoulder_z"]
        angles = [45.0, 90.0]
        self.visualizer.set_angles(joints, angles)
        # check if robot config matches target angles
        target_cfg = dict(zip(joints, map(math.radians, angles)))
        self.assertEqual(self.visualizer.robot.cfg, target_cfg)

    def test_set_angles_radians(self):
        """Checks if setting multiple angles as radians works"""
        # set two angles in radian
        joints = ["r_shoulder_y", "r_shoulder_z"]
        angles = [0.79, 1.57]
        self.visualizer.set_angles_radians(joints, angles)
        # check if robot config matches target angles
        target_cfg = dict(zip(joints, angles))
        self.assertEqual(self.visualizer.robot.cfg, target_cfg)

    def test_target_position(self):
        """Checks if target node is set correctly in the scene"""
        # set target position
        self.visualizer.set_target_position(0.4, 0.3, 0.5)
        # check if translation part of the target nodes frame matches position
        frame = self.visualizer.robot.scene.get_pose(self.visualizer.robot.target_node)
        translation = frame[:, 3][:3]
        self.assertEqual(list(translation), [0.4, 0.3, 0.5])

    def test_target_frame(self):
        """Tests setting a target frame directly"""
        # set target frame
        rpyM = geometry.rpy_matrix(0.79, 1.57, 3.14)
        target_frame = geometry.to_transformation_matrix([0.4, 0.3, 0.5], rpyM)
        self.visualizer.set_target_frame(target_frame)
        # check if frame of the target node matches target frame
        frame = self.visualizer.robot.scene.get_pose(self.visualizer.robot.target_node)
        assert_array_equal(frame, target_frame)


if __name__ == "__main__":
    unittest.main()
