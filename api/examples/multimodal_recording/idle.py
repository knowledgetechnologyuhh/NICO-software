# -*- coding: utf-8 -*-
"""
Created on Mon Mar 12 14:59:12 2018

@author: kerzel
"""

import sys
import serial
import numpy as np
import time
from time import sleep
import subprocess
from subprocess import call 
import random
import datetime
import os

from nicomotion import Motion
from nicoface import FaceExpression

robot = Motion.Motion("/informatik3/wtm/home/kerzel/Documents/NICOdialogue/json/nico_humanoid_legged_with_hands_mod.json",vrep=False)
robot.disableTorqueAll()
