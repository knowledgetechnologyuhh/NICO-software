Usage
===

> **Note:** Due to changes in the setup, versions of the API installed prior to 22.09.2020 should be removed. Please delete the `~/.NICO`, `api/build` and `api/devel` folders before running the setup script.

To setup a virtualenv, install most required packages and build ros, use one of
the two setup scripts.

For Python 2.7:

```
source NICO-setup.bash
```

For Python 3:

```
source NICO-python3.bash
```

Either setup will generate an activation script that can be sourced to use the API without reinstalling or updating packages:

```
source api/activate.bash
```

Prerequisites
===
Most prerequesites are automatically installed by sourcing `NICO-setup.bash` or
`NICO-python3.bash`. However some environment variables and permissions need to be set and apt packages to be installed for all the libraries to work.

PyRep
--

PyRep, which can be used to control and interact with CoppeliaSim, is automatically installed by the setup, but requires `COPPELIASIM_ROOT` to point to the folder where CoppeliaSim is located. To do so, edit the path and add this to your `.bashrc`:
```
export COPPELIASIM_ROOT=EDIT/ME/PATH/TO/COPPELIASIM/INSTALL/DIR
```

Additionally, the `COPPELIASIM_ROOT` has to be added to `LD_LIBRARY_PATH` and `QT_QPA_PLATFORM_PLUGIN_PATH` to run PyRep. Since these can cause conflicts with other programms, a bash script is provided that can be sourced whenever PyRep is needed:

```
source pyrep_env.bash
```

Alternatively, add the following to your `.bashrc`
(**note that this can cause conflicts with other programms**):
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
```

See https://github.com/stepjam/PyRep for more information about PyRep.


nicoaudio
--

`PyAudio` requires `portaudio`:
```
sudo apt-get install portaudio19-dev
```
`PyDub` requires `ffmpeg` to process non-wav files:
```
sudo apt-get install ffmpeg
```
TextToSpeech:

The `nicoaudio.TextToSpeech` uses `pico2wave` as fallback (e.g if there is no internet):
```
sudo apt-get install pico2wave
```
It can also use a locally hosted [MozillaTTS server](https://github.com/mozilla/TTS/tree/master/server) (note that MozillaTTS is still in developement)

nicomotion
--
To open a serial connection, you need permissions for the port. This can be
solved by adding the user to the dialout group:
```
sudo adduser $USER dialout
```
otherwise you need to manually set the permissions each time you open a shell:
```
sudo chmod 777 /dev/ttyACM*
```

`nicomotion.Motion` also tries to adjust port latency using `setserial`:
```
sudo apt-get install setserial
```

nicoros
--

For informations on ROS see https://www.ros.org/


nicovision
--

v4l2-ctl:
```
sudo apt-get install v4l-utils
```

nicotouch
--
To open a serial connection, you need permissions for the port. This can be
solved by adding the user to the dialout group:
```
sudo adduser $USER dialout
```
otherwise you need to manually set the permissions each time you open a shell:
```
sudo chmod 777 /dev/ttyACM*
```

Testing
===
To verify whether the installation was successful, you can execute the following test script:

```
source NICO-test.bash
```

This script will run a series of tests for the `nicoaudio`, `nicoface` and `nicomotion` modules. Note that it will only test functions independant of external hardware. Tests for the `nicomotion` require `pyrep`.

If you only want to test a specific module, you can also run `pytest` directly
in the corresponding test directory, e.g. `nicoaudio`:

```
pytest -v src/nicoaudio/tests
```
