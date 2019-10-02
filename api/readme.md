Prerequesits
===
Most prerequesits are automatically installed by sourcing `NICO-setup.bash` or
`NICO-python3.bash`, however there are some apt packages that need to be
installed for the libraries to work.

nicoaudio
--

PyAudio:
```
sudo apt-get install portaudio19-dev
```
PyDub:
```
sudo apt-get install ffmpeg
```
TextToSpeech:

The TTS module uses pico2wave as fallback:
```
sudo apt-get install pico2wave
```
It can also use a locally hosted [MozillaTTS server](https://github.com/mozilla/TTS/tree/master/server) (note that MozillaTTS is still in developement)

nicomotion
--
If possible (permantent solution, relog after command):
```
sudo adduser $USER dialout
```
otherwise (each time you open a shell):
```
sudo chmod 777 /dev/ttyACM*
```

setserial:
```
sudo apt-get install setserial
```

PyRep:

If you want to use PyRep to cotrol V-Rep, add the following to .bashrc (see [official repository](https://github.com/stepjam/PyRep) for more info):
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

```
sudo apt-get install python-catkin-tools
```
Source in addition to `api/devel/setup.bash` and `~/.NICO-python3/bin/activate` (`NICO-python3.bash` does all of this automatically, consider adding catkin workspaces to .bashrc):
```
source cv_bridge_build_ws/devel/setup.bash --extend
```

nicotouch
--
If possible (permantent solution, relog after command):
```
sudo adduser $USER dialout
```
otherwise (each time you open a shell):
```
sudo chmod 777 /dev/ttyACM*
```
