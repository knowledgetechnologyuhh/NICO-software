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
    package_data={
        "": [
            "scripts/nicoemotionrecognition/_nicoemotionrecognition_internal/Trained\ Networks/*"
        ]
    },
    include_package_data=True,
    description="NICO api package for Pablo Barros' emotion recognition",
    author="Connor Gaede",
    author_email="4gaede@informatik.uni-hamburg.de",
    install_requires=["tensorflow", "keras==2.2.4", "dlib"],
    **extra
)
