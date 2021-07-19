#!/bin/bash

# parameters
: ${PYTHON="/usr/bin/python3"} # python executable of virtualenv
: ${VIRTUALENVDIR="NICO-python3"} # directory name of virtualenv in home (with a '.' prefix)
: ${SKIP_PYREP=0} # set to 1 to ignore pyrep environment checks
: ${REINSTALL_PYPOT=0} # Set this to 1 to force a reinstall of pypot

# get dir
CALLDIR=$(pwd)
WORKDIR="`dirname "$BASH_SOURCE"`"

# check if COPPELIASIM_ROOT is set properly
if  [ -z "$COPPELIASIM_ROOT" ]; then
  if [ $SKIP_PYREP -eq 0 ]; then
    echo -e "\e[31mERROR: COPPELIASIM_ROOT not set, which is required for PyRep installation (see readme.md). To continue regardless, set SKIP_PYREP to 1.\e[0m"
    return 1 2> /dev/null
  else
    echo -e "\e[33mWARNING: COPPELIASIM_ROOT not set, PyRep will not be installed.\e[0m"
  fi
elif ! [ -f "$COPPELIASIM_ROOT/coppeliaSim.sh" ]; then
  echo -e "\e[31mERROR: Could not find 'coppeliaSim.sh' inside COPPELIASIM_ROOT directory ($COPPELIASIM_ROOT).\e[0m"
  return 1 2> /dev/null
fi

# run NICO setup
SKIP_CLEANUP=1 # skips cleanup in NICO-setup.bash. DO NOT SET THIS MANUALLY
source $WORKDIR/NICO-setup.bash
if ! [ $? -eq 0 ]; then
  return 1 2> /dev/null
fi

# build cv bridge for python3 (for ros versions before noetic)
if [ ! -z $ROS_DISTRO ] && [ $ROS_DISTRO != noetic ]; then
  if [ ! -f $WORKDIR/../cv_bridge_build_ws/devel/setup.bash ]; then
    echo "Building cv_bridge for python 3"
    pip install catkin_tools
    mkdir $WORKDIR/../cv_bridge_build_ws
    cd $WORKDIR/../cv_bridge_build_ws
    mkdir src
    catkin config -DPYTHON_EXECUTABLE=~/.$VIRTUALENVDIR/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
    catkin config --install
    cd src
    git clone -b melodic https://github.com/ros-perception/vision_opencv.git
    cd $WORKDIR/../cv_bridge_build_ws
    catkin build cv_bridge --cmake-args -DPYTHON_VERSION=3.6
  fi
  if [ -f $WORKDIR/../cv_bridge_build_ws/devel/setup.bash ]; then
    source $WORKDIR/../cv_bridge_build_ws/devel/setup.bash --extend
  else
    echo -e "\e[33mWARNING: Custom cv_bridge not found - cv_bridge won't work under python 3\e[0m"
  fi
fi

# cleanup
unset SKIP_CLEANUP
unset SKIP_PYREP
cleanup
