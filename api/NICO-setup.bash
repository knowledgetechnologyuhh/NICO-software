#!/bin/bash

# get dir
DIR=`dirname "$BASH_SOURCE"`
WORKDIR=`pwd`
echo Running at: $WORKDIR/$DIR

#ROS + catkin
echo "Setting up API"
cd $WORKDIR/$DIR
if [ -x "$(command -v catkin_make)" ]; then
  catkin_make
  source $WORKDIR/$DIR/devel/setup.bash
fi
if ! [ -x "$(command -v catkin_make)" ]; then
  echo "Catkin not found - skipping API building"
fi

#cleanup
echo "Cleanup"
cd "$WORKDIR"

echo "done"
echo
