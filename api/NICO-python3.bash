#!/bin/bash
: ${PYTHON="/usr/bin/python3"}
: ${VIRTUALENVDIR="NICO-python3"}
: ${SKIP_PYREP=0}
CALLDIR=$(pwd)
WORKDIR="`dirname "$BASH_SOURCE"`"
SKIP_CLEANUP=1

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

source $WORKDIR/NICO-setup.bash
if ! [ $? -eq 0 ]; then
  return 1 2> /dev/null
fi

if [ ! -z $ROS_DISTRO ] && [ $ROS_DISTRO != noetic ]; then
  if [ -x "$(command -v catkin)" ] && ! [ -f $WORKDIR/../cv_bridge_build_ws/devel/setup.bash ]; then
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

unset SKIP_CLEANUP
unset SKIP_PYREP
cleanup
