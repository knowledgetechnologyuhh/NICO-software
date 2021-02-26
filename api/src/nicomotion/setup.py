#!/usr/bin/env python
import sys
from setuptools import find_packages, setup

extra = {}
if sys.version_info >= (3,):
    extra["use_2to3"] = True

setup(
    name="nicomotion",
    version="1.0",
    packages=find_packages("scripts/"),
    package_dir={"": "scripts"},
    package_data={"": ["scripts/nicomotion/urdf/*"]},
    include_package_data=True,
    description="NICO api package for motion related modules",
    author="Connor Gaede",
    author_email="4gaede@informatik.uni-hamburg.de",
    install_requires=["math3d", "transforms3d", "numpy", "matplotlib", "gaikpy",],
    **extra
)
