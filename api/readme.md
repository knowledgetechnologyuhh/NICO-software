Usage
===
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

Prerequisites
===
Most prerequesites are automatically installed by sourcing `NICO-setup.bash` or
`NICO-python3.bash`, however there are some apt packages that need to be
installed for the libraries to work.

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

PyRep:

If you want to use PyRep to control V-Rep, add the following to .bashrc (see [official repository](https://github.com/stepjam/PyRep) for more info):
```
export VREP_ROOT=EDIT/ME/PATH/TO/V-REP/INSTALL/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$VREP_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$VREP_ROOT
```

nicovision
--

v4l2-ctl:
```
sudo apt-get install v4l-utils
```
nicoros
--

cv-bridge for python 3:

As of writing, there is no pre-compiled `cv-bridge` for python 3. Therefore the
`NICO-python3.bash` script needs to build it, which requires `catkin build`:
```
sudo apt-get install python-catkin-tools
```

`cv_bridge_build_ws` needs to be sourced in addition to `api/devel/setup.bash`
and `~/.NICO-python3/bin/activate`. (note that `NICO-python3.bash` does this automatically):
```
source cv_bridge_build_ws/devel/setup.bash --extend
```

You might want to consider adding both catkin workspace setups to your
`.bashrc` so you only need to activate the existing virtualenv if you do not
want to execute `NICO-python3.bash`.

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
