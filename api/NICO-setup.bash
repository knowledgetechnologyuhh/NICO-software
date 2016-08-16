#!/bin/bash

# get dir
DIR=`dirname "$BASH_SOURCE"`
WORKDIR=`pwd`
VIRTUALENV="virtualenv"
echo Running at: $WORKDIR/$DIR

#install pypot patched version
cd
if [ -d ".NICO/" ]; then
    echo "PyPot already installed"
    source ~/.NICO/bin/activate
else
  echo "Installing patched pypot"
  # Test for virtualenv
  command -v virtualenv
  if ! [ -x "$(command -v foo)" ]; then
    pip install --user virtualenv
    VIRTUALENV=".local/bin/virtualenv"
  fi
  $VIRTUALENV -p /usr/bin/python2.7 --system-site-packages ~/.NICO
  source ~/.NICO/bin/activate
  pip install pypot
  cd /tmp
  git clone https://git.informatik.uni-hamburg.de/2soll/pypot.git
  rm -rf ~/.NICO/lib/python2.7/site-packages/pypot/
  mv /tmp/pypot/pypot ~/.NICO/lib/python2.7/site-packages/
fi

#ROS + catkin
echo "Setting up API"
cd $WORKDIR/$DIR
catkin_make
source $WORKDIR/$DIR/devel/setup.bash

#cleanup
echo "Cleanup"
rm -rf /tmp/pypot
cd $WORKDIR

echo "done"
echo