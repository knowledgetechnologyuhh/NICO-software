#!/bin/bash

# get dir
CALLDIR=`pwd`
cd "`dirname "$BASH_SOURCE"`"
WORKDIR=$(pwd)
cd "$CALLDIR"
VIRTUALENV="virtualenv"
echo Running at: "$WORKDIR"

: ${PYTHON="/usr/bin/python2.7"}
: ${VIRTUALENVDIR="NICO"}
if [ $# -eq 1 ]
  then
    VIRTUALENVDIR="$1"
fi
echo Virtual environment directory: "$VIRTUALENVDIR"

cd

# Test for network connection
for interface in $(ls /sys/class/net/ | grep -v lo);
do
  if [[ $(cat /sys/class/net/$interface/carrier) = 1 ]]; then ONLINE=1; break; fi
done

#virtualenv setup
echo "Checking for virtualenv"
if [ -d ".$VIRTUALENVDIR/" ]; then
    echo "Existing virtualenv found"
else
  echo "No virtualenv found - setting up new virtualenv"
  # Test for virtualenv
  if ! [ -x "$(command -v virtualenv)" ]; then
    pip install --user virtualenv
    VIRTUALENV=".local/bin/virtualenv"
  fi
  $VIRTUALENV -p $PYTHON ~/.$VIRTUALENVDIR
fi
echo "Activating virtualenv"
source ~/.$VIRTUALENVDIR/bin/activate

#install python packages
if [ $ONLINE ] && [ $VIRTUAL_ENV == ~/.$VIRTUALENVDIR ]; then
  
  echo "Checking python packages"
  pip install 'sphinx' # required inside virtualenv to find all modules
  pip install cffi
  # install/update custom pypot
  cd /tmp
  git clone https://git.informatik.uni-hamburg.de/wtm-robots-and-equipment/pypot.git
  cd pypot
  CURRENT_GIT_COMMIT=`git show --name-status | grep commit`
  CURRENT_GIT_COMMIT=${CURRENT_GIT_COMMIT#'commit '}
  if [ ! -f ~/.$VIRTUALENVDIR/.current_git_commit ] || [ ! `cat ~/.$VIRTUALENVDIR/.current_git_commit` == $CURRENT_GIT_COMMIT ]; then
    echo "Custom pypot outdated - updating to commit $CURRENT_GIT_COMMIT"
    rm -rf ~/.$VIRTUALENVDIR/lib/python*/site-packages/pypot/
    pip install .
    echo $CURRENT_GIT_COMMIT >| ~/.$VIRTUALENVDIR/.current_git_commit
  else
    echo "Latest custom pypot already installed - skipping installation"
  fi
  for api_package in nicoaudio nicoemotionrecognition nicoface nicomotion nicotouch nicovision; do
    pip install $WORKDIR/src/$api_package/
  done
else
  if [ ! $ONLINE ]; then
    echo "Not connected to the internet - skipping python package installations"
  else
    echo "Activation failed - skipping python package installations"
  fi
fi

#MoveIt!
MOVEIT_indigo=$(dpkg-query -W --showformat='${Status}\n' ros-indigo-moveit 2>/dev/null|grep "install ok
installed")
MOVEIT_kinetic=$(dpkg-query -W --showformat='${Status}\n' ros-kinetic-moveit 2>/dev/null|grep "install ok
installed")
MOVEIT_melodic=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-moveit 2>/dev/null|grep "install ok
installed")
if [ "" == "$MOVEIT_indigo" ] && [ "" == "$MOVEIT_kinetic" ] && [ "" == "$MOVEIT_melodic" ]; then
  if [ -f $WORKDIR/src/nicomoveit/kinematics/package.xml ]; then
    rm $WORKDIR/src/nicomoveit/kinematics/package.xml
  fi
  echo "MoveIt! is not installed"
else
  if [ -f $WORKDIR/src/nicomoveit/kinematics/package_.xml ]; then
    cp $WORKDIR/src/nicomoveit/kinematics/package_.xml $WORKDIR/src/nicomoveit/kinematics/package.xml
  fi
  echo "MoveIt! is installed"
  echo "To use MoveIt! with visualization run: roslaunch nicoros nicoros_moveit_visual.launch"
  echo "To use MoveIt! without visualization run: roslaunch nicoros nicoros_moveit.launch"
  pip install 'pyassimp==4.1.3' #FIXME version 4.1.4 causes segmentation faults while loading stl files
fi

#ROS + catkin
echo "Setting up API"
cd $WORKDIR
if [ -e /opt/ros/indigo/setup.bash ]; then
  ROS_VERSION="indigo"
  source /opt/ros/${ROS_VERSION}/setup.bash
elif [ -e /opt/ros/kinetic/setup.bash ]; then
  ROS_VERSION="kinetic"
  source /opt/ros/${ROS_VERSION}/setup.bash
elif [ -e /opt/ros/melodic/setup.bash ]; then
  ROS_VERSION="melodic"
  source /opt/ros/${ROS_VERSION}/setup.bash
fi
if [ -x "$(command -v catkin_make)" ]; then
  pip install rospkg catkin_pkg empy
  catkin_make -DPYTHON_EXECUTABLE=~/.$VIRTUALENVDIR/bin/python
  source $WORKDIR/devel/setup.bash
fi
if ! [ -x "$(command -v catkin_make)" ]; then
  echo "Catkin not found - skipping API building"
fi

#cleanup
echo "Cleanup"
rm -rf /tmp/pypot
cd "$CALLDIR"

echo "done"
echo
