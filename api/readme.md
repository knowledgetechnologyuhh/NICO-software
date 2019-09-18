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
**WIP**

cv-bridge for python 3:

```
sudo apt-get install python-catkin-tools
```
```
mkdir opencv_build_ws
cd opencv_build_ws
catkin config -DPYTHON_EXECUTABLE=~/.NICO-python3/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin config --install

mkdir src
cd src
git clone -b melodic https://github.com/ros-perception/vision_opencv.git

cd ..
catkin build cv_bridge
source install/setup.bash --extend
```
