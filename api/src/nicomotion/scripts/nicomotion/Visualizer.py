import math

from os.path import dirname, abspath
from gaikpy import robot
from ikpy.utils import geometry


class Visualizer(object):
    """The Visualizer allows to pre-visualize Robot motor angles."""

    def __init__(self, urdf=(dirname(abspath(__file__)) + "/urdf/kinematics.urdf")):
        """
        Allows to visualize and manipulate joint angles of the given urdf

        :param urdf: path of the urdf file
        :type urdf: str
        """
        self.robot = robot.robot(urdf, [])

    def set_angle(self, name, angle):
        """
        Set angle of the specified motor

        :param name: motor name
        :type name: str
        :param angle: motor angle in degrees
        :type angle: float
        """
        self.set_angles([name], [angle])

    def set_angles(self, names, angles):
        """
        Set list of motors to specified angles

        :param names: list of motor names
        :type names: list(str)
        :param angles: list of angles in degrees
        :type angles: list(float)
        """
        angles = map(math.radians, angles)
        self.set_angles_radians(names, angles)

    def set_angles_radians(self, names, angles):
        """
        Set list of motors to specified angles in radians

        :param names: list of motor names
        :type names: list(str)
        :param angles: list of angles in radians
        :type angles: list(float)
        """

        self.robot.active_chain_definition = names
        self.robot.update_robot_pose(angles)

    def set_target_frame(self, target_frame):
        """
        Visualize position and orientation represented by the given
        4x4 transformation matrix

        :param target_frame: 4x4 transformation matrix
        :type target_frame: np.array
        """
        self.robot.update_target(target_frame)

    def set_target_position(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Visualize given position and orientation

        :param x: position x (forward)
        :type x: float
        :param y: position y (horizontal)
        :type y: float
        :param z: position z (vertical)
        :type z: float
        :param roll: rotation around x-axis in degree
        :type roll: float
        :param pitch: rotation around y-axis in degree
        :type pitch: float
        :param yaw: rotation around z-axis in degree
        :type yaw: float
        """
        roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])
        rpyM = geometry.rpy_matrix(roll, pitch, yaw)
        target_frame = geometry.to_transformation_matrix([x, y, z], rpyM)
        self.set_target_frame(target_frame)
