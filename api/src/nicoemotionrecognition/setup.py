#!/usr/bin/env python
import sys
import subprocess
from os import dirname, abspath
from setuptools import find_packages, setup

extra = {}
if sys.version_info >= (3,):
    extra["use_2to3"] = True

setup(
    name="nicoemotionrecognition",
    version="1.0",
    packages=find_packages("scripts/"),
    package_dir={"": "scripts"},
    include_package_data=True,
    description="NICO api package for Pablo Barros' emotion recognition",
    author="Connor Gaede",
    author_email="gaede@informatik.uni-hamburg.de",
    install_requires=[
        "flaskcom @ git+https://github.com/LemonSpeech/flaskcom.git@44660c8#egg=flaskcom"
    ],
    **extra
)

if subprocess.getstatusoutput("command -v docker")[0] == 0:
    subprocess.call(
        "docker build -t emotionrecognition {}".format(
            dirname(abspath(__file__))
            + "/scripts/nicoemotionrecognition/_nicoemotionrecognition_internal"
        ),
        shell=True,
    )
else:
    print("\033[31mDocker not installed - cannot build emotion recognition server\033")
