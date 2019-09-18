#!/usr/bin/env python

from setuptools import find_packages, setup

setup(
    name="nicoemotionrecognition",
    version="1.0",
    packages=find_packages("scripts/"),
    package_dir={'': 'scripts'},
    include_package_data=True,
    description="NICO api package for Pablo Barros' emotion recognition",
    author='Connor Gaede',
    author_email='4gaede@informatik.uni-hamburg.de',
)
