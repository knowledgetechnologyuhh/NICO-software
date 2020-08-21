import cv2
import logging
from datetime import datetime
from nicovision.PyrepDevice import PyrepDevice
from pyrep.backend import sim

logger = logging.getLogger(__name__)


class PyrepRecorder(object):
    """
    PyrepRecorder allows recording images and videos from CoppeliaSim
    vision sensors using pyrep.
    """

    @staticmethod
    def autodetect_nicoeyes():
        """
        Returns a tuple containing the detected names of left and right eye
        sensor

        :return: Devicenames as (left, right) tuple (None if not found)
        :rtype: tuple
        """
        return PyrepDevice.autodetect_nicoeyes()

    def __init__(
        self,
        sensor_names=[],
        width=640,
        height=480,
        near_clipping_plane=1e-2,
        far_clipping_plane=10.0,
        view_angle=60.0,
    ):
        """
        Initializes recorder that can capture images and videos from one or
        more vision sensors in a CoppeliaSim scene.

        :param sensor_names: name/s (or handle/s) of the vision sensor/s in the
                             scene
        :type sensor_names: str/list(str)
        :param width: horizontal camera resolution
        :type width: int
        :param heigth: vertical camera resolution
        :type heigth: int
        :param near_clipping_plane: the minimum distance from which the sensors
                                    will be able to detect.
        :type near_clipping_plane: float
        :param far_clipping_plane: the maximum distance from which the sensors
                                   will be able to detect.
        :type far_clipping_plane: float
        :param view_angle: field of view of the camera in degree.
        :type view_angle: float
        """
        logger.info("Initialising vision sensors")
        self._image_callback_ids = []
        self._video_callback_ids = []
        self._devices = []
        if not sensor_names:
            logger.info("No sensor names given - trying to autodetect")
            for name in PyrepDevice.autodetect_nicoeyes():
                if name:
                    self._devices.append(
                        PyrepDevice(
                            name,
                            width,
                            height,
                            near_clipping_plane,
                            far_clipping_plane,
                            view_angle,
                        )
                    )
            if not self._devices:
                logger.error("Failed to autodetect vision sensors")
                raise ValueError(
                    "Got empty 'sensor_names' and could not autodetect eye "
                    + "sensors in scene"
                )
        elif isinstance(sensor_names, str):
            self._devices.append(
                PyrepDevice(
                    sensor_names,
                    width,
                    height,
                    near_clipping_plane,
                    far_clipping_plane,
                    view_angle,
                )
            )
        elif isinstance(sensor_names, (list, tuple)):
            for name in sensor_names:
                self._devices.append(
                    PyrepDevice(
                        name,
                        width,
                        height,
                        near_clipping_plane,
                        far_clipping_plane,
                        view_angle,
                    )
                )

    @property
    def sensor_names(self):
        """
        :return: names of the recorded sensors
        :rtype: str
        """
        return [d.sensor_name for d in self._devices]

    def step(self):
        """
        Triggers step method of all recorded sensors
        """
        for device in self._devices:
            device.step()

    def preprocess_image(self, frame):
        """
        Can be overriden to preprocess each image.

        :param frame: the frame to preprocess.
        :type frame: cv2.image
        :return: the preprocessed frame
        :rtype: cv2.image
        """
        return frame

    def create_image_callback(
        self, sensor_name, filename, timestamp_format="%Y-%m-%d_%H%M%S%f"
    ):
        """
        Creates a callback to save each frame as an image.

        :param sensor_name: sensor_name that is inserted into the filename
        :type sensor_name: str
        :param filename: filename with optional 'sensor_name' and 'timestamp'
                         placeholders.
        :type filename: str
        :param timestamp_format: formatstring for the timestamp
        :type timestamp_format: str
        :return: the callback function
        :rtype: function
        """

        def callback(frame):
            frame = self.preprocess_image(frame)
            cv2.imwrite(
                filename.format(
                    sensor_name=sensor_name,
                    timestamp=datetime.now().strftime(timestamp_format),
                ),
                frame,
            )

        return callback

    def start_image_recording(
        self,
        filename="{sensor_name}_image_{timestamp}.png",
        timestamp_format="%Y-%m-%d_%H%M%S%f",
    ):
        """
        Starts recording which saves a png image at each simulation step.

        :param filename: filename with optional 'sensor_name' and 'timestamp'
                         placeholders.
        :type filename: str
        :param timestamp_format: formatstring for the timestamp
        :type timestamp_format: str
        """

        if self._image_callback_ids:
            logger.warn("Cannot start multiple image recordings.")
        else:
            # create callback for each sensor
            for device in self._devices:
                callback = self.create_image_callback(device.sensor_name, filename)
                callback_id = device.add_callback(callback)
                self._image_callback_ids.append(callback_id)

    def stop_image_recording(self):
        """
        Stops recording of images.
        """

        if not self._image_callback_ids:
            logger.warn("There is no running image recording to stop.")
        else:
            # remove callbacks and clear id's
            for device_id in range(len(self._devices)):
                self._devices[device_id].remove_callback(
                    self._image_callback_ids[device_id]
                )
            self._image_callback_ids.clear()

    def take_one_image(
        self,
        filename="{sensor_name}_image_{timestamp}.png",
        timestamp_format="%Y-%m-%d_%H%M%S%f",
    ):
        """
        Takes a single image at the current simulation step.

        :param filename: filename with optional 'sensor_name' and 'timestamp'
                         placeholders.
        :type filename: str
        :param timestamp_format: formatstring for the timestamp
        :type timestamp_format: str
        """

        for device in self._devices:
            frame = device.get_image()
            frame = self.preprocess_image(frame)
            cv2.imwrite(
                filename.format(
                    sensor_name=device.sensor_name,
                    timestamp=datetime.now().strftime(timestamp_format),
                ),
                frame,
            )

    def create_video_callback(self, writer):
        """
        Creates a callback to add each frame to a video.

        :param writer: videowriter to write the frames
        :type sensor_name: cv2.VideoWriter
        :return: the callback function
        :rtype: function
        """

        def callback(frame):
            writer.write(frame)

        return callback

    def start_video_recording(
        self,
        filename="{sensor_name}_video_{timestamp}.avi",
        fourcc="DIVX",
        timestamp_format="%Y-%m-%d_%H%M%S%f",
    ):
        """
        Starts video recording that records each simulation step.

        :param filename: filename with optional 'sensor_name' and 'timestamp'
                         placeholders.
        :type filename: str
        :param fourcc: The fourcc of the video codec.
        :type fourcc: str
        :param timestamp_format: formatstring for the timestamp
        :type timestamp_format: str
        """

        if self._video_callback_ids:
            logger.warn("Cannot start multiple video recordings.")
        else:
            # calculate fps from timestep
            fps = round(1.0 / sim.simGetSimulationTimeStep())
            # codec
            fourcc = cv2.VideoWriter_fourcc(*fourcc)
            # setup video writer for each sensor
            for device in self._devices:
                writer = cv2.VideoWriter(
                    filename.format(
                        sensor_name=device.sensor_name,
                        timestamp=datetime.now().strftime(timestamp_format),
                    ),
                    fourcc,
                    fps,
                    device.resolution,
                )
                # add callback
                callback = self.create_video_callback(writer)
                callback_id = device.add_callback(callback)
                self._video_callback_ids.append(callback_id)

    def stop_video_recording(self):
        """
        Stops the video recording.
        """

        if not self._video_callback_ids:
            logger.warn("There is no running video recording to stop.")
        else:
            # remove callbacks and clear id's
            for device_id in range(len(self._devices)):
                self._devices[device_id].remove_callback(
                    self._video_callback_ids[device_id]
                )
            self._video_callback_ids.clear()
