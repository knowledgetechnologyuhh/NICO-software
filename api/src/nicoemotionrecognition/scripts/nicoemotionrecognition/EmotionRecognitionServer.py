#!/usr/bin/env python

import docker
import logging
import os
import subprocess

logger = logging.getLogger(__name__)


class EmotionRecognitionServer(object):
    """Starts emotion recognition model inside Docker."""

    @property
    def tags(self):
        return ("master", "development")

    def __init__(self, gui, tag="master"):
        docker_registry = "git.informatik.uni-hamburg.de:4567"
        docker_image = "{}/wtm-robots-and-equipment/nico-software/emotionrecognition".format(
            docker_registry
        )
        client = docker.from_env()
        # check if docker image with tag exists locally
        try:
            client.images.get("{}:{}".format(docker_image, tag))
        except docker.errors.ImageNotFound:
            logger.warn(
                "No local image with tag '{}' found - pulling from registry".format(tag)
            )
            # pull image from docker repository
            try:
                client.images.pull(docker_image, tag)
            except docker.errors.NotFound as e:
                # default to master if no branch version is available
                if tag != "master":
                    logger.warn(
                        "Could not pull image with tag {} - defaulting to master"
                    )
                    tag = "master"
                    try:
                        client.images.get("{}:{}".format(docker_image, tag))
                    except docker.errors.ImageNotFound:
                        client.images.pull(docker_image, tag)
                else:
                    raise e
            except docker.errors.APIError as e:
                logger.error("Error while pulling docker image: {}".format((str(e))))
                if "access forbidden" in str(e):
                    logger.error("Make sure you have access to the docker repository:")
                    logger.error(" docker login {}".format(docker_registry))
                exit(1)

        # enable x forwarding or add disable gui command
        if gui:
            logger.info("enabling gui")
            subprocess.run(["xhost", "+"])
            cmd = ""
        else:
            logger.info("disabling gui")
            cmd = "--disable-gui"
        # start docker container
        logger.info("Starting docker container")
        client.containers.run(
            "{}:{}".format(docker_image, tag),
            command=cmd,
            auto_remove=True,
            environment=["DISPLAY=" + os.environ["DISPLAY"]],
            ipc_mode="host",
            network_mode="host",
        )


if __name__ == "__main__":
    EmotionRecognitionServer(gui=True)
