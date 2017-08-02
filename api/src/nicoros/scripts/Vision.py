#!/usr/bin/env python

import logging
import argparse
import rospy
import sensor_msgs.msg
import cv_bridge
import nicovision.VideoDevice
import sys

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
            'device': '',
            'framerate': 20,
            'width': 640,
            'height': 480,
            'rostopicName': '/nico/vision'
        }

    def __init__(self, config = None):
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

        logging.info('-- Init NicoRosMotion --')
        rospy.init_node('nicorosvision', anonymous=True)

        logging.debug('Init ROS publisher')
        self._publisher = rospy.Publisher(self._config['rostopicName'] + '/videoStream', sensor_msgs.msg.Image, queue_size = 1)

        logging.info('-- All done --')
        pass

    def startStream(self):
        """
        Starts the stream
        """
        if self._stream_running:
            logging.warning('Stream already running')
            return
        self._device = nicovision.VideoDevice.VideoDevice.fromDevice(self._config['device'])
        if self._device is None:
            logging.error('Can not initialise device - is the device name correct and not ambiguous?')
            return
        self._device.addCallback(self._callback)
        self._device.setFrameRate(self._config['framerate'])
        self._device.setResolution(self._config['width'], self._config['height'])
        self._device.open()
        self._stream_running = True

    def stopStream(self):
        """
        Stops the stream
        """
        if not self._stream_running:
            logging.warning('Stream not running')
            return
        self._device.close()
        self._device = None
        self._stream_running = False

    def isRunning(self):
        """
        Returns true if stream is currently running

        :return: true if running
        :rtype: bool
        """
        return self._stream_running

    def _callback(self, rval, frame):
        """
        Callback for device

        :param rval: rval
        :param frame: frame
        """
        if frame is not None:
            self._publisher.publish(self._bridge.cv2_to_imgmsg(frame, 'bgr8'))

if __name__ == '__main__':
    config = NicoRosVision.getConfig()

    parser = argparse.ArgumentParser(description='NICO ROS vision interface')
    parser.add_argument('--log-level', dest='logLevel', help='Sets log level. Default: INFO', type=str, default='INFO')
    parser.add_argument('--log-file', dest='logFile', help='Path to log file. Default: NICO_VISION.log', type=str, default='NICO_VISION.log')
    parser.add_argument('-d', '--device', dest='device', help='Target device. Default: %s' % config['device'], type=str)
    parser.add_argument('-f', '--framerate', dest='framerate', help='Capture framerate. Default: %i' % config['framerate'], type=int)
    parser.add_argument('-W', '--width', dest='width', help='Image width. Default: %i' % config['width'], type=float)
    parser.add_argument('-H', '--height', dest='height', help='Image height. Default: %i' % config['height'], type=float)
    parser.add_argument('--rostopic-name', dest='rostopicName', help='Topic name for ROS. Default: %s' % config['rostopicName'], type=str)

    args = parser.parse_known_args()[0]

    # Parse args
    if args.device:
        config['device'] = args.device
    if args.framerate:
        config['framerate'] = args.framerate
    if args.width:
        config['width'] = args.width
    if args.height:
        config['height'] = args.height
    if args.rostopicName:
        config['rostopicName'] = args.rostopicName

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
        sys.stderr.write('LOGGING ERROR: Unknown log level %s\n' % args.logLevel)
        pass

    logging.basicConfig(filename=args.logFile,
                        format='%(asctime)s %(levelname)s at %(funcName)s (%(module)s: %(lineno)d): %(message)s',
                        level=loggingLevel)
    stdoutHandler = logging.StreamHandler(sys.stdout)
    stdoutHandler.setLevel(loggingLevel)
    logging_format = logging.Formatter(
        '%(asctime)s %(levelname)s at %(funcName)s (%(module)s: %(lineno)d): %(message)s')
    stdoutHandler.setFormatter(logging_format)
    logging.getLogger().addHandler(stdoutHandler)

    vision = NicoRosVision(config)

    vision.startStream()
    rospy.spin()
    vision.stopStream()