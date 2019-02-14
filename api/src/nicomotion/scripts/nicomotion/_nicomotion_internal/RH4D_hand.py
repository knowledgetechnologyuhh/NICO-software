from hand import AbstractHand


class RH4DHand(AbstractHand):
    """This class represents the Seed Robotics RH4D Hand."""

    current_limit = 250

    sensitive_motors = ("thumb_x", "indexfingers_x")

    current_ports = {"wrist_z": "present_current_port_1",
                     "wrist_x": "present_current_port_2",
                     "thumb_x": "present_current_port_3",
                     "indexfingers_x": "present_current_port_4"}

    poses = {"openHand": {"thumb_x": (-160., 1.),
                          "indexfingers_x": (-160., 1.), },
             "closeHand": {"thumb_x": (100., 1.),
                           "indexfingers_x": (100., 1.), },
             "openHandVREP": {"thumb_x": (0., 1.),
                              "indexfingers_x": (0., 1.), },
             "closeHandVREP": {"thumb_x": (-30., 1.),
                               "indexfingers_x": (-30., 1.), },
             }

    def __init__(self, robot, isLeft, monitorCurrents=True):
        super(RH4DHand, self).__init__(robot, isLeft, monitorCurrents)
