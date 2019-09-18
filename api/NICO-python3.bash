#!/bin/bash
export PYTHON=/usr/bin/python3
export VIRTUALENVDIR=NICO-python3
CALLDIR=$(pwd)
echo $CALLDIR
cd "`dirname "$BASH_SOURCE"`"
WORKDIR=$(pwd)
cd $CALLDIR
echo $WORKDIR
source $WORKDIR/NICO-setup.bash
if ! [ -d $WORKDIR/../cv_bridge_build_ws/install/lib/python3/dist-packages/cv_bridge ]; then
  echo "Building cv_bridge for python 3"
  mkdir $WORKDIR/../cv_bridge_build_ws
  cd $WORKDIR/../cv_bridge_build_ws
  catkin config -DPYTHON_EXECUTABLE=~/.$VIRTUALENVDIR/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
  catkin config --install
  mkdir src
  cd src
  git clone -b melodic https://github.com/ros-perception/vision_opencv.git
  cd $WORKDIR/../cv_bridge_build_ws
  catkin build cv_bridge
fi
unset PYTHON
unset VIRTUALENVDIR
cd $WORKDIR/..
echo $(pwd)
source cv_bridge_build_ws/install/setup.bash
source cv_bridge_build_ws/install/setup.bash
echo $CALLDIR
cd $CALLDIR
