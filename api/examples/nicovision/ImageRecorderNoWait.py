# Simple Example using multiple instances of ImageRecorder
#
# NOTE Only do this if absolutely necessary.  If possible, try to use
# the nicovision.MultiCamRecorder module instead, which opens and
# closes all devices at the same time
#
# Authors:
# Connor

import datetime
import logging
import os
import resource
import sys
from os.path import abspath, dirname
from time import sleep

import objgraph
from nicovision import ImageRecorder
from nicovision.VideoDevice import VideoDevice
from pympler import tracker

logging.basicConfig(level=logging.DEBUG)

if not os.path.isdir(dirname(abspath(__file__)) + '/recorded_images'):
    os.mkdir(dirname(abspath(__file__)) + '/recorded_images')

res_x = 1920
res_y = 1080
framerate = 30
amount_of_cams = 2
zoom_level = 150
pan = 0
tilt = 0
devices = VideoDevice.autodetect_nicoeyes()
calibration_file = dirname(abspath(__file__)) + \
    "/../../../json/nico_vision_calibration_params.json"

logging.debug("devices: {}".format(", ".join(devices)))

recordings = 20
mem_usage = [0] * recordings
tr = tracker.SummaryTracker()
for recording in range(recordings):
    for i in range(amount_of_cams):
        if not os.path.isdir(dirname(abspath(__file__)) +
                             '/recorded_images/camera{}'.format(i)):
            os.mkdir(dirname(abspath(__file__)) +
                     '/recorded_images/camera{}'.format(i))

    ir = ImageRecorder.ImageRecorder(
        devices[0], res_x, res_y, zoom=zoom_level, pan=pan, tilt=tilt,
        framerate=framerate, writer_threads=2, pixel_format="UYVY",
        calibration_file=calibration_file)
    # disable writing during recording to save CPU if necessary
    ir.enable_write(state=False)

    if amount_of_cams >= 2:
        ir2 = ImageRecorder.ImageRecorder(
            devices[1], res_x, res_y, zoom=zoom_level, pan=pan, tilt=tilt,
            framerate=framerate, writer_threads=2, pixel_format="UYVY",
            calibration_file=calibration_file)
        # disable writing during recording to save CPU if necessary
        ir2.enable_write(state=False)

    # NOTE only use seperate ImageRecorder instances if absolutely necessary.
    # If possible use nicovision.MultiCamRecorder for synchronized multi-camera
    # recordings:
    #
    # ir = MultiCamRecorder.MultiCamRecorder(devices, res_x, res_y,
    #                                        framerate=framerate,
    #                                        zoom=zoom_level,
    #                                        pan=pan,
    #                                        tilt=tilt,
    #                                        writer_threads=4,
    #                                        pixel_format="UYVY")
    sleep(2)

    logging.info("Start taking pictures")
    logging.debug(datetime.datetime.today().isoformat())

    ir.start_recording(path=dirname(abspath(__file__)) +
                       '/recorded_images/camera0/picture-{}.png')
    if amount_of_cams == 2:
        ir2.start_recording(path=dirname(abspath(__file__)) +
                            '/recorded_images/camera1/picture-{}.png')

    # For MultiCamRecorder first {} is replaced with camera id:
    #
    # ir.start_recording(path=dirname(abspath(__file__)) +
    #                    '/recorded_images/camera{}/picture-{}.png')
    sleep(10)
    logging.info("Stop recording")
    # stop image writer without waiting for all images. WARNING this causes
    # memory leaks if the writer isn't properly closed later on
    ir.stop_recording(wait_for_writer=False)
    # ir2 will wait for all images to be written
    ir2.stop_recording()
    # wait for ir's writer to finish (necessary to avoid memory leaks)
    ir.wait_for_writer()
    # for MultiCamRecorder call:
    # ir.stop_recording()
    ir = None
    ir2 = None
    logging.info("Finished taking pictures")
    mem_usage[recording] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    logging.debug("Memory usage: {} (kb)".format(mem_usage[recording]))
    if recording > 0:
        logging.debug("Memory diff to last iteration: {}".format(
            mem_usage[recording] - mem_usage[recording - 1]))
    logging.debug("Memory diff total: {}".format(
        mem_usage[recording] - mem_usage[0]))
    logging.debug("Memory diff per object: ")
    tr.print_diff()
    logging.debug("New Objects:")
    objgraph.show_growth()
