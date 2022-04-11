#!/usr/bin/env python

import subprocess


class EmotionRecognitionServer(object):
    """Starts emotion recognition model inside Docker."""

    def __init__(self, gui):
        # command to run docker container
        docker_cmd = (
            "docker run  --rm -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v"
            + ' /tmp/.X11-unix:/tmp/.X11-unix --env="QT_X11_NO_MITSHM=1" '
            + "emotionrecognition",
        )
        # enable x-forwarding or append disable gui option
        if gui:
            subprocess.call("xhost +", shell=True)
        else:
            docker_cmd += " --disable-gui"
        # start docker container
        subprocess.call(docker_cmd, shell=True)


if __name__ == "__main__":
    EmotionRecognitionServer(gui=True)
