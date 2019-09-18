#!/usr/bin/env python

from setuptools import find_packages, setup

setup(
    name="nicoface",
    version="1.0",
    packages=find_packages("scripts/"),
    package_dir={'': 'scripts'},
    description='NICO api package for face expressions',
    author='Connor Gaede',
    author_email='4gaede@informatik.uni-hamburg.de',
)
