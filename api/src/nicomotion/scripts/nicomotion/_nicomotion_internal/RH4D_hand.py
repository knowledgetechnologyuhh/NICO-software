from .hand import AbstractHand


class RH4DHand(AbstractHand):
    """This class represents the Seed Robotics RH4D Hand."""

    current_limit = 250

    sensitive_motors = ("thumb_x", "indexfingers_x")

    current_ports = {"wrist_z": 0, "wrist_x": 1, "thumb_x": 2, "indexfingers_x": 3}

    poses = {
        "openHand": {"thumb_x": (-160.0, 1.0), "indexfingers_x": (-160.0, 1.0)},
        "closeHand": {"thumb_x": (100.0, 1.0), "indexfingers_x": (100.0, 1.0)},
        "openHandVREP": {"thumb_x": (0.0, 1.0), "indexfingers_x": (0.0, 1.0)},
        "closeHandVREP": {"thumb_x": (-30.0, 1.0), "indexfingers_x": (-30.0, 1.0)},
    }

    conversion_angles = {
        "l_wrist_x": ((-180, 180), (-50, 0)),
        "r_wrist_x": ((-180, 180), (0, 50)),
        "l_wrist_z": ((-180, 180), (-90, 90)),
        "r_wrist_z": ((-180, 180), (-90, 90)),
        "l_indexfingers_x": ((-160, 160), (-75, 0)),
        "r_indexfingers_x": ((-160, 160), (-75, 0)),
        "l_thumb_x": ((-160, 160), (-75, 0)),
        "r_thumb_x": ((-160, 160), (-75, 0)),
    }

    def __init__(self, robot, isLeft, monitorCurrents=True, vrep=False):
        super(RH4DHand, self).__init__(robot, isLeft, monitorCurrents, vrep)
