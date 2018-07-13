#!/usr/bin/env python

import argparse
import logging
import sys

import cv_bridge
import nicomsg.srv
import nicovision.MultiCamRecorder as MultiCamRecorder
import rospy
import sensor_msgs.msg


class NicoRosVision():
    """
    The NicoRosVision class exposes a camera stream over ROS
    """

    @staticmethod
    def getConfig():
        """
        Returns a default config dict

        :return: dict
        """
        return {
            'mode': 'stereo',
            'device_left': '',
            'device_right': '',
            'framerate': 20,
            'width': 640,
            'height': 480,
            'zoom': None,
            'pan': None,
            'tilt': None,
            'settings_file': None,
            'setting': "standard",
            'rostopic_prefix': '/nico/vision',
            'rostopic_left': '/left',
            'rostopic_right': '/right'
        }

    def __init__(self, config=None):
        """
        The NicoRosVision enables the sending of a camera image through ROS

        :param config: Configuration dict
        :type config: dict
        """
        self._device = None
        self._stream_running = False
        self._config = config
        if config is None:
            self._config = NicoRosVision.getConfig()
        self._bridge = cv_bridge.CvBridge()

        logging.info('-- Init NicoRosVision --')
        rospy.init_node('nicorosvision', anonymous=True)
        logging.debug('Init ROS publishers')
        self._publishers = []
        if self._config['mode'] in ("stereo", "left"):
            self._publishers.append(rospy.Publisher(
                self._config['rostopic_prefix'] +
                self._config['rostopic_left'],
                sensor_msgs.msg.Image, queue_size=1))
        if self._config['mode'] in ("stereo", "right"):
            self._publishers.append(rospy.Publisher(
                self._config['rostopic_prefix'] +
                self._config['rostopic_right'],
                sensor_msgs.msg.Image, queue_size=1))

        logging.debug('Init ROS services')
        rospy.Service(
            '%s/setZoom' % config['rostopic_prefix'], nicomsg.srv.SetIntValue,
            self._ROSPY_setZoom)
        rospy.Service(
            '%s/setPan' % config['rostopic_prefix'], nicomsg.srv.SetIntValue,
            self._ROSPY_setPan)
        rospy.Service(
            '%s/setTilt' % config['rostopic_prefix'], nicomsg.srv.SetIntValue,
            self._ROSPY_setTilt)
        logging.info('-- All done --')

    def _ROSPY_setZoom(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.setZoom`

        :param message: ROS message
        :type message: nicomsg.srv.SetIntValue
        :return: success
        :rtype: bool
        """
        if self._device is None:
            logging.warning("No video device initialized")
            return False
        return self._device.zoom(message.value)

    def _ROSPY_setPan(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.setPan`

        :param message: ROS message
        :type message: nicomsg.srv.SetIntValue
        :return: success
        :rtype: bool
        """
        if self._device is None:
            logging.warning("No video device initialized")
            return False
        return self._device.pan(message.value)

    def _ROSPY_setTilt(self, message):
        """
        Callback handle for :meth:`nicomotion.Motion.setTilt`

        :param message: ROS message
        :type message: nicomsg.srv.SetIntValue
        :return: success
        :rtype: bool
        """
        if self._device is None:
            logging.warning("No video device initialized")
            return False
        return self._device.tilt(message.value)

    def start_stream(self):
        """
        Starts the stream
        """
        if self._stream_running:
            logging.warning('Stream already running')
            return

        nico_eyes = MultiCamRecorder.autodetect_nicoeyes()
        devices = []
        if self._config['mode'] in ("stereo", "left"):
            if self._config["device_left"] == '':
                devices.append(nico_eyes[0])
            else:
                devices.append(self._config['device_left'])
        if self._config['mode'] in ("stereo", "right"):
            if self._config["device_right"] == '':
                devices.append(nico_eyes[1])
            else:
                devices.append(self._config['device_right'])
        self._device = \
            MultiCamRecorder.MultiCamRecorder(devices,
                                              self._config['width'],
                                              self._config['height'],
                                              self._config['framerate'],
                                              self._config['zoom'],
                                              self._config['pan'],
                                              self._config['tilt'],
                                              self._config['settings_file'],
                                              self._config['setting'],
                                              writer_threads=0,
                                              pixel_format="UYVY")
        if self._device is None:
            logging.error(
                'Can not initialise device - is the device name correct ' +
                'and not ambiguous?')
            return
        self._device.add_callback(self._callback)
        self._stream_running = True

    def stop_stream(self):
        """
        Stops the stream
        """
        if not self._stream_running:
            logging.warning('Stream not running')
            return
        self._device.stop_recording()
        self._stream_running = False

    def is_running(self):
        """
        Returns true if stream is currently running

        :return: true if running
        :rtype: bool
        """
        return self._stream_running

    def _callback(self, rval, frame, id):
        """
        Callback for device

        :param rval: rval
        :param frame: frame
        """
        if frame is not None:
            self._publishers[id].publish(
                self._bridge.cv2_to_imgmsg(frame, 'bgr8'))


if __name__ == '__main__':
    config = NicoRosVision.getConfig()

    parser = argparse.ArgumentParser(description='NICO ROS vision interface')
    parser.add_argument('--log-level', dest='logLevel',
                        help='Sets log level. Default: INFO', type=str,
                        default='INFO')
    parser.add_argument('--log-file', dest='logFile',
                        help='Path to log file. Default: NICO_VISION.log',
                        type=str, default='NICO_VISION.log')
    parser.add_argument('-m', '--mode', dest='mode',
                        help='Streaming mode. (left, right, stereo) ' +
                        'Default: {}'.format(config['mode']),
                        type=str)
    parser.add_argument('-l', '--device-left', dest='device_left',
                        help='Left recording device. Autodetected if not set',
                        type=str)
    parser.add_argument('-r', '--device-right', dest='device_right',
                        help='Right recording device. Autodetected if not set',
                        type=str)
    parser.add_argument('-f', '--framerate', dest='framerate',
                        help='Capture framerate. Default: %i' %
                        config['framerate'], type=int)
    parser.add_argument('-W', '--width', dest='width',
                        help='Image width. Default: %i' % config['width'],
                        type=float)
    parser.add_argument('-H', '--height', dest='height',
                        help='Image height. Default: %i' % config['height'],
                        type=float)
    parser.add_argument('--zoom', dest='zoom',
                        help='Camera zoom (if supported). Default: None',
                        type=int)
    parser.add_argument('--pan', dest='pan',
                        help='Camera pan (if supported). Default: None',
                        type=int)
    parser.add_argument('--tilt', dest='tilt',
                        help='Camera tilt (if supported). Default: None',
                        type=int)
    parser.add_argument('--settings-file', dest='settings_file',
                        help='Path to camera settings json. Default: None',
                        type=str)
    parser.add_argument('--setting', dest='setting',
                        help='Name of setting in settings json. (if any)' +
                        'Default: {}'.format(config['setting']),
                        type=str)
    parser.add_argument('--rostopic-prefix', dest='rostopic_prefix',
                        help='ROS topic prefix for both devices. Default: %s' %
                        config['rostopic_prefix'], type=str)
    parser.add_argument('--rostopic-left', dest='rostopic_left',
                        help='ROS topic name for left device. Default: %s' %
                        config['rostopic_left'], type=str)
    parser.add_argument('--rostopic-right', dest='rostopic_right',
                        help='ROS topic name for right device. Default: %s' %
                        config['rostopic_right'], type=str)

    args = parser.parse_known_args()[0]

    # Parse args
    if args.mode:
        config['mode'] = args.mode
    if args.device_left:
        config['device_left'] = args.device_left
    if args.device_right:
        config['device_right'] = args.device_right
    if args.framerate:
        config['framerate'] = args.framerate
    if args.width:
        config['width'] = args.width
    if args.height:
        config['height'] = args.height
    if args.zoom:
        config['zoom'] = args.zoom
    if args.pan:
        config['pan'] = args.pan
    if args.tilt:
        config['tilt'] = args.tilt
    if args.settings_file:
        config['settings_file'] = args.settings_file
    if args.setting:
        config['setting'] = args.setting
    if args.rostopic_prefix:
        config['rostopic_prefix'] = args.rostopic_prefix
    if args.rostopic_left:
        config['rostopic_left'] = args.rostopic_left
    if args.rostopic_right:
        config['rostopic_right'] = args.rostopic_right

    # Set logging setting
    loggingLevel = logging.INFO
    try:
        loggingLevel = {
            'DEBUG': logging.DEBUG,
            'INFO': logging.INFO,
            'WARNING': logging.WARNING,
            'CRITICAL': logging.CRITICAL,
            'debug': logging.DEBUG,
            'info': logging.INFO,
            'warning': logging.WARNING,
            'critical': logging.CRITICAL,
        }[args.logLevel]
    except:
        sys.stderr.write(
            'LOGGING ERROR: Unknown log level %s\n' % args.logLevel)
        pass

    logging.basicConfig(filename=args.logFile,
                        format='%(asctime)s %(levelname)s at %(funcName)s ' +
                        '(%(module)s: %(lineno)d): %(message)s',
                        level=loggingLevel)
    stdoutHandler = logging.StreamHandler(sys.stdout)
    stdoutHandler.setLevel(loggingLevel)
    logging_format = logging.Formatter(
        '%(asctime)s %(levelname)s at %(funcName)s (%(module)s: ' +
        '%(lineno)d): %(message)s')
    stdoutHandler.setFormatter(logging_format)
    logging.getLogger().addHandler(stdoutHandler)

    vision = NicoRosVision(config)
    vision.start_stream()
    rospy.spin()
    vision.stop_stream()
