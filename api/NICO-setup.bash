#!/bin/bash

# parameters
: ${PYTHON="/usr/bin/python2.7"} # python executable of virtualenv
: ${VIRTUALENV_DIR=$HOME} # where to create the virtualenv
: ${VIRTUALENV_NAME="NICO"} # directory name of virtualenv in $VIRTUALENVROOT (with a '.' prefix)
: ${REINSTALL_PYPOT=0} # Set this to 1 to force a reinstall of pypot
if ! [ -z $VIRTUALENVDIR ]; then
  echo -e "\e[31mERROR: VIRTUALENVDIR is no longer supported, please set VIRTUALENV_DIR and VIRTUALENV_NAME instead.\e[0m"
  return 1 2> /dev/null
fi
if [ $# -eq 1 ]
then
  VIRTUALENV_NAME="$1"
fi
echo Virtual environment directory: "$VIRTUALENV_DIR/.$VIRTUALENV_NAME"

# get dir
CALLDIR=`pwd`
cd "`dirname "$BASH_SOURCE"`"
WORKDIR=$(pwd)
cd "$CALLDIR"
VIRTUALENV="virtualenv"
echo Running at: "$WORKDIR"

cd

cleanup() {
  echo "Cleanup"
  unset PYTHON
  unset VIRTUALENV_DIR
  unset VIRTUALENV_NAME
  unset REINSTALL_PYPOT
  rm -rf /tmp/pypot
  cd "$CALLDIR"
}
# Test for network connection
for interface in $(ls /sys/class/net/ | grep -v lo);
do
  if [[ $(cat /sys/class/net/$interface/carrier) = 1 ]]; then ONLINE=1; break; fi
done

#virtualenv setup
echo "Checking for virtualenv"
if [ -d "$VIRTUALENV_DIR/.$VIRTUALENV_NAME/" ]; then
  echo "Existing virtualenv found"
else
  echo "No virtualenv found - setting up new virtualenv"
  # Test for virtualenv
  if ! [ -x "$(command -v $VIRTUALENV)" ]; then
    echo -e "\e[31mERROR: Could not find command $VIRTUALENV, please make sure it is installed or change the value of VIRTUALENV to the proper command.\e[0m"
  fi
  $VIRTUALENV -p $PYTHON $VIRTUALENV_DIR/.$VIRTUALENV_NAME/
fi
echo "Activating virtualenv"
source $VIRTUALENV_DIR/.$VIRTUALENV_NAME/bin/activate

#install python packages
if [ $ONLINE ] && [ $VIRTUAL_ENV == $VIRTUALENV_DIR/.$VIRTUALENV_NAME ]; then

  echo "Checking python packages"
  pip install 'sphinx' # required inside virtualenv to find all modules
  pip install cffi # pyrep requirement
  pip install 'empy<4'

  # install/update custom pypot
  CURRENT_GIT_COMMIT=`git ls-remote https://github.com/knowledgetechnologyuhh/pypot.git HEAD | awk '{ print $1}'`
  if [ $REINSTALL_PYPOT == 1 ] || [ ! -f ~/.$VIRTUALENVDIR/.current_git_commit ] || [ ! `cat ~/.$VIRTUALENVDIR/.current_git_commit` == $CURRENT_GIT_COMMIT ]; then
    echo "Custom pypot outdated - updating to commit $CURRENT_GIT_COMMIT"
    cd /tmp
    git clone https://github.com/knowledgetechnologyuhh/pypot.git
    if [ ! $? -eq 0 ]; then
      echo -e "\e[31mERROR: Could not clone pypot\e[0m"
      cleanup
      return 1 2> /dev/null
    fi
    cd pypot
    pip uninstall pypot -y
    pip install .
    if [ ! -z "$(pip list --disable-pip-version-check | grep -F pypot)" ]
    then
      echo $CURRENT_GIT_COMMIT >| $VIRTUALENV_DIR/.$VIRTUALENV_NAME/.current_git_commit
    else
      cleanup
      echo -e "\e[31mERROR: Could not install pypot\e[0m"
      return 1 2> /dev/null
    fi
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

# ROS

# source setup
for DISTRO in indigo kinetic melodic noetic
do
  if [ -e /opt/ros/${DISTRO}/setup.bash ]; then
    source /opt/ros/${DISTRO}/setup.bash
    break
  fi
done

# MoveIt!
if ! [ -z $ROS_DISTRO ]; then
  MOVEIT=$(dpkg-query -W --showformat='${Status}\n' ros-${ROS_DISTRO}-moveit
  2>/dev/null | grep "install ok installed")
  if [ -z "$MOVEIT" ]; then
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
    pip install defusedxml
    pip install netifaces
    pip install pyside2
  fi
fi

# build catkin workspace
echo "Building ROS packages"
cd $WORKDIR

if [ -x "$(command -v catkin_make)" ]; then
  pip install rospkg catkin_pkg empy
  catkin_make -DPYTHON_EXECUTABLE=$VIRTUALENV_DIR/.$VIRTUALENV_NAME/bin/python
  source $WORKDIR/devel/setup.bash
fi
if ! [ -x "$(command -v catkin_make)" ]; then
  echo "Catkin not found - skipping ROS packages"
fi

# activation script
echo "Generating 'activate.bash'"
if [ -f activate.bash ]; then
  rm activate.bash
fi
BRIDGE_PATH=$(realpath $WORKDIR/..)/cv_bridge_build_ws/devel/setup.bash
cat <<END > activate.bash
#!/bin/bash

# activate python environment
source $VIRTUALENV_DIR/.$VIRTUALENV_NAME/bin/activate

# activate ros workspace
if [ ! -z $ROS_DISTRO ] && [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
  source /opt/ros/$ROS_DISTRO/setup.bash
  source $(realpath $WORKDIR/devel/setup.bash)
  if [ -f $BRIDGE_PATH ]; then
    source $BRIDGE_PATH --extend
  fi
fi
END

# cleanup
if [[ -x SKIP_CLEANUP ]]; then
  cleanup
fi

echo "done"
echo
