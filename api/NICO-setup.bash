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
  pip install 'sphinx' # required inside virtualenv to find all modules
  # install/update custom pypot
  cd /tmp
  git clone https://git.informatik.uni-hamburg.de/wtm-robots-and-equipment/pypot.git
  cd pypot
  CURRENT_GIT_COMMIT=`git show --name-status | grep commit`
  CURRENT_GIT_COMMIT=${CURRENT_GIT_COMMIT#'commit '}
  if [ ! -f ~/.NICO/.current_git_commit ] || [ ! `cat ~/.NICO/.current_git_commit` == $CURRENT_GIT_COMMIT ]; then
    echo "Custom pypot outdated - updating to commit $CURRENT_GIT_COMMIT"
    rm -rf ~/.NICO/lib/python2.7/site-packages/pypot/
    ~/.NICO/bin/python setup.py install
    echo $CURRENT_GIT_COMMIT >| ~/.NICO/.current_git_commit
  else
    echo "Latest custom pypot already installed - skipping installation"
  fi
  pip install 'pyassimp'
  pip install pyassimp --upgrade
else
  echo "Activation failed - skipping python package installations"
fi

#MoveIt!
MOVEIT_indigo=$(dpkg-query -W --showformat='${Status}\n' ros-indigo-moveit 2>/dev/null|grep "install ok
installed")
MOVEIT_kinetic=$(dpkg-query -W --showformat='${Status}\n' ros-kinetic-moveit 2>/dev/null|grep "install ok
installed")
if [ "" == "$MOVEIT_indigo" ] && [ "" == "$MOVEIT_kinetic" ]; then
  echo "MoveIt! is not installed"
else
  if [ -f $WORKDIR/$DIR/src/nicomoveit/kinematics/package_.xml ]; then
    mv $WORKDIR/$DIR/src/nicomoveit/kinematics/package_.xml $WORKDIR/$DIR/src/nicomoveit/kinematics/package.xml
  fi
  echo "MoveIt! is installed"
  echo "To use MoveIt! with visualization run: roslaunch nicoros nicoros_moveit_visual.launch"
  echo "To use MoveIt! without visualization run: roslaunch nicoros nicoros_moveit.launch"
fi

#ROS + catkin
echo "Setting up API"
cd $WORKDIR/$DIR
if [ -e /opt/ros/indigo/setup.bash ]; then
  ROS_VERSION="indigo"
  source /opt/ros/${ROS_VERSION}/setup.bash
elif [ -e /opt/ros/kinetic/setup.bash ]; then
  ROS_VERSION="kinetic"
  source /opt/ros/${ROS_VERSION}/setup.bash
fi
if [ -x "$(command -v catkin_make)" ]; then
  catkin_make
  source $WORKDIR/$DIR/devel/setup.bash
fi
if ! [ -x "$(command -v catkin_make)" ]; then
  echo "Catkin not found - skipping API building"
fi

#cleanup
echo "Cleanup"
rm -rf /tmp/pypot
cd "$WORKDIR"

echo "done"
echo
