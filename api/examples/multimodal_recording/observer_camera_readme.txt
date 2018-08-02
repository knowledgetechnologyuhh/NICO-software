How to start the observer camera server on wtmpc211:

- check if both cameras are connected (usb3)
- check if the camera device order is correct_
  realsense r200 should be video0, video1, video2
  brio HD cam should be video3

- login as sysadmin

in first terminal 
- source /opt/ros/kinetic/setup.bash
- roslaunch realsense_camera r200_nodelet_nico.launch

in second terminal

- cd /export/NICO/NICO-software/api/examples/multimodal_recording
- ./start_observer_server.sh

