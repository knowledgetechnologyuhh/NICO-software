#!/usr/bin/env python
import sys
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
        "docker",
        "flaskcom @ git+https://github.com/LemonSpeech/flaskcom.git@44660c8#egg=flaskcom",
    ],
    **extra
)
