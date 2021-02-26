import math
import unittest

from ikpy.utils import geometry
from nicomotion import Visualizer
from numpy.testing import assert_array_equal


class VisualizerTest(unittest.TestCase):
    def setUp(self):
        self.visualizer = Visualizer.Visualizer()

    def test_set_angle(self):
        joint = "r_shoulder_z"
        angle = 45.0
        self.visualizer.set_angle(joint, angle)
        self.assertEqual(self.visualizer.robot.cfg, {joint: math.radians(angle)})

    def test_set_angles(self):
        joints = ["r_shoulder_y", "r_shoulder_z"]
        angles = [45.0, 90.0]
        self.visualizer.set_angles(joints, angles)
        target_cfg = dict(zip(joints, map(math.radians, angles)))
        self.assertEqual(self.visualizer.robot.cfg, target_cfg)

    def test_set_angles_radians(self):
        joints = ["r_shoulder_y", "r_shoulder_z"]
        angles = [0.79, 1.57]
        self.visualizer.set_angles_radians(joints, angles)
        target_cfg = dict(zip(joints, angles))
        self.assertEqual(self.visualizer.robot.cfg, target_cfg)

    def test_target_position(self):
        self.visualizer.set_target_position(0.4, 0.3, 0.5)
        frame = self.visualizer.robot.scene.get_pose(self.visualizer.robot.target_node)
        translation = frame[:, 3][:3]
        self.assertEqual(list(translation), [0.4, 0.3, 0.5])

    def test_target_frame(self):
        rpyM = geometry.rpy_matrix(0.79, 1.57, 3.14)
        target_frame = geometry.to_transformation_matrix([0.4, 0.3, 0.5], rpyM)
        self.visualizer.set_target_frame(target_frame)
        frame = self.visualizer.robot.scene.get_pose(self.visualizer.robot.target_node)
        assert_array_equal(frame, target_frame)


if __name__ == "__main__":
    unittest.main()
