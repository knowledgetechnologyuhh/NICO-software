#!/bin/bash

# sets the paths required for pyrep, only requires COPPELIASIM_ROOT to be set
if  [ -z "$COPPELIASIM_ROOT" ]; then
  echo -e "\e[31mERROR: COPPELIASIM_ROOT not set. Please set it to the directory where 'coppeliaSim.sh' is located.\e[0m"
  return 1 2> /dev/null
elif ! [ -f "$COPPELIASIM_ROOT/coppeliaSim.sh" ]; then
  echo -e "\e[31mERROR: Could not find 'coppeliaSim.sh' inside COPPELIASIM_ROOT directory ($COPPELIASIM_ROOT).\e[0m"
  return 1 2> /dev/null
else
  if ! [[ $LD_LIBRARY_PATH == *$COPPELIASIM_ROOT* ]]; then
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
  fi
  export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
fi
