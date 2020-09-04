#!/usr/bin/env python
import sys
from setuptools import find_packages, setup

extra = {}
if sys.version_info >= (3,):
    extra["use_2to3"] = True

setup(
    name="nicoface",
    version="1.0",
    packages=find_packages("scripts/"),
    package_dir={"": "scripts"},
    description="NICO api package for face expressions",
    author="Connor Gaede",
    author_email="4gaede@informatik.uni-hamburg.de",
    install_requires=["matplotlib", "numpy", "pillow", "posix_ipc"],
    **extra
)
