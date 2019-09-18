from .hand import AbstractHand


class RH5DHand(AbstractHand):
    """This class represents the Seed Robotics RH4D Hand."""

    current_limit = 175

    sensitive_motors = ("thumb_z", "thumb_x",
                        "indexfinger_x", "middlefingers_x")

    current_ports = {"wrist_z": 0,
                     "thumb_z": 3,
                     "thumb_x": 4,
                     "indexfinger_x": 5,
                     "middlefingers_x": 6}

    poses = {"pointAt": {"indexfinger_x": (-170., 1.),
                         "middlefingers_x": (150., 1.),
                         "thumb_x": (60., .6),
                         "thumb_z": (150., 1.)},
             "openHand": {"indexfinger_x": (-150., 1.),
                          "middlefingers_x": (-150., 1.),
                          "thumb_x": (-150., 1.),
                          "thumb_z": (-150., 1.)},
             "closeHand": {"indexfinger_x": (120., 1.),
                           "middlefingers_x": (120., 1.),
                           "thumb_x": (60., .6),
                           "thumb_z": (150., 1.)},
             "thumbsUp": {"indexfinger_x": (150., 1.),
                          "middlefingers_x": (150., 1.),
                          "thumb_x": (-150., 1.),
                          "thumb_z": (-150., 1.)},
             "okSign": {"indexfinger_x": (-25., 1.),
                        "middlefingers_x": (-170., 1.),
                        "thumb_x": (-25., 1.),
                        "thumb_z": (45., 1.)},
             "pinchToIndex": {"indexfinger_x": (-25., 1.),
                              "middlefingers_x": (150., 1.),
                              "thumb_x": (-25., 1.),
                              "thumb_z": (45., 1.)},
             "keyGrip": {"indexfinger_x": (30., 1.),
                         "middlefingers_x": (150., 1.),
                         "thumb_x": (30., 1.),
                         "thumb_z": (-170., 1.)},
             "pencilGrip": {"indexfinger_x": (90., .5),
                            "middlefingers_x": (150., 1.),
                            "thumb_x": (60., .6),
                            "thumb_z": (150., 1.)}
             }

    def __init__(self, robot, isLeft, monitorCurrents=True, vrep=False):
        super(RH5DHand, self).__init__(robot, isLeft, monitorCurrents, vrep)
