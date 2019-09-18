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
```
sudo apt-get install pico2wave
```
mozilla tts?

nicomotion
--

PyRep:
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
Source in addition to api/devel/setup.bash and NICO-python3/bin/activate (NICO-python3.bash does all of this automatically):
```
source cv_bridge_build_ws/devel/setup.bash --extend
```
