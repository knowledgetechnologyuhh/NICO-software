#!/bin/bash

# get dir
DIR=`dirname "$BASH_SOURCE"`
WORKDIR=`pwd`
VIRTUALENV="virtualenv"
echo Running at: $WORKDIR/$DIR

cd

#test if we have to reset the setup
if [ -d ".NICO/" ]; then
    if [ ! -f ".NICO/.$CURRENT_GIT_COMMIT" ]; then
        echo "Resetting enviroment"
        rm -r ".NICO/"
    fi
fi

#install pypot version
if [ -d ".NICO/" ]; then
    echo "PyPot already installed"
    source ~/.NICO/bin/activate
else
  echo "Installing pypot"
  # Test for virtualenv
  if ! [ -x "$(command -v virtualenv)" ]; then
    pip install --user virtualenv
    VIRTUALENV=".local/bin/virtualenv"
  fi
  $VIRTUALENV -p /usr/bin/python2.7 --system-site-packages ~/.NICO
  source ~/.NICO/bin/activate
  pip install 'pyserial==3.1'
  pip install 'pypot>=3.0.0'
fi

#Saving CURRENT_GIT_COMMIT
cd
touch ".NICO/.$CURRENT_GIT_COMMIT"

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
