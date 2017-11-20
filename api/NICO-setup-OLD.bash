#!/bin/bash

# get dir
DIR=`dirname "$BASH_SOURCE"`
WORKDIR=`pwd`
VIRTUALENV="virtualenv"
echo Running at: $WORKDIR/$DIR

cd

#virtualenv setup
echo "Checking for virtualenv"
if [ -d ".NICO/" ]; then
    echo "Existing virtualenv found"
else
  echo "No virtualenv found - setting up new virtualenv"
  # Test for virtualenv
  if ! [ -x "$(command -v virtualenv)" ]; then
    pip install --user virtualenv
    VIRTUALENV=".local/bin/virtualenv"
  fi
  $VIRTUALENV -p /usr/bin/python2.7 --system-site-packages ~/.NICO
fi
echo "Activating virtualenv"
source ~/.NICO/bin/activate

#install python packages
if [ $VIRTUAL_ENV == ~/.NICO ]; then
  echo "Checking python packages"
  pip install 'pyserial<=3.1' # versions 3.2 and 3.3 (most recent as of writing) are missing __init__.pyc for tools
  pip install 'pypot>=3.0.0'
  pip install 'sphinx' # required inside virtualenv to find all modules
else
  echo "Activation failed - skipping python package installations"
fi

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
