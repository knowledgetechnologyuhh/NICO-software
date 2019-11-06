#!/bin/bash
export PYTHON=/usr/bin/python3
export VIRTUALENVDIR=NICO-python3
CALLDIR=$(pwd)
WORKDIR="`dirname "$BASH_SOURCE"`"
INSTALL_PYREP=1

cleanup_vars() {
  unset PYTHON
  unset VIRTUALENVDIR
}

: ${IGNORE_MISSING_VREP=0}
# check if VREP_ROOT is set properly
if  [ -z "$VREP_ROOT" ]; then
  if [ -d /informatik3/wtm/public/installations/VREP/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04 ]; then
    echo "Using shared V-REP installation"
    export VREP_ROOT=/informatik3/wtm/public/installations/VREP/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$VREP_ROOT
    export QT_QPA_PLATFORM_PLUGIN_PATH=$VREP_ROOT
  elif [ $IGNORE_MISSING_VREP -eq 0 ]; then
    echo -e "\e[31mERROR: VREP_ROOT not set, which is required for PyRep installation (see readme.md). To continue regardless, set IGNORE_MISSING_VREP to 1.\e[0m"
    return 1 2> /dev/null
  else
    echo -e "\e[33mWARNING: VREP_ROOT not set, PyRep will not be installed.\e[0m"
  fi
elif ! [ -f "$VREP_ROOT/vrep.sh" ]; then
  echo -e "\e[31mERROR: Could not find 'vrep.sh' inside VREP_ROOT directory ($VREP_ROOT).\e[0m"
  return 1 2> /dev/null
fi

source $WORKDIR/NICO-setup.bash
if ! [ $? -eq 0 ]; then
  cleanup_vars
  return 1 2> /dev/null
fi

if [ -x "$(command -v catkin)" ] && ! [ -f $WORKDIR/../cv_bridge_build_ws/devel/setup.bash ]; then
  echo "Building cv_bridge for python 3"
  mkdir $WORKDIR/../cv_bridge_build_ws
  cd $WORKDIR/../cv_bridge_build_ws
  mkdir src
  catkin config -DPYTHON_EXECUTABLE=~/.$VIRTUALENVDIR/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
  catkin config --install
  cd src
  git clone -b melodic https://github.com/ros-perception/vision_opencv.git
  cd $WORKDIR/../cv_bridge_build_ws
  catkin build cv_bridge
fi
if [ -f $WORKDIR/../cv_bridge_build_ws/devel/setup.bash ]; then
  source $WORKDIR/../cv_bridge_build_ws/devel/setup.bash --extend
else
  echo -e "\e[33mWARNING: Custom cv_bridge not found - cv_bridge won't work under python 3\e[0m"
fi

cleanup_vars
