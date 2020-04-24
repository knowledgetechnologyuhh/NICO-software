import inspect
import logging
import numpy as np
import weakref
from pyrep import PyRep
from pyrep.objects.vision_sensor import VisionSensor

logger = logging.getLogger(__name__)

step_func = PyRep.step  # save original step function


def step_with_listeners(self):
    """
    Triggers StepListener step function after executing simulation step
    """
    step_func(self)  # execute PyRep step
    StepListener.step()


PyRep.step = step_with_listeners  # override step function


class StepListener:
    """
    StepListener is a static class that allows to register objects as listeners
    whose step functions are executed after each simulation step.
    """

    _subscribers = []  # static list, do not override!

    @staticmethod
    def step():
        """
        Executes the step() method of all registered objects.
        """
        for sub in StepListener._subscribers:
            sub().step()

    @staticmethod
    def register(obj_wref):
        """
        Registers an weak-referenced object as a listener. The object needs to
        have a step() method, which is called each time a simulation step is
        executed.

        :param obj_wref: weakref to register as listener. Needs to have a step
                         method.
        :type obj_wref: weakref.ref
        """
        if isinstance(obj_wref, weakref.ref):
            if hasattr(obj_wref(), "step"):
                StepListener._subscribers.append(obj_wref)
            else:
                logger.error("Trying to register object without step method")
                raise ValueError("Trying to register object without step method")
        else:
            logger.error("Trying to register non-weakref object")
            raise ValueError("Trying to register non-weakref object")

    @staticmethod
    def unregister(obj_wref):
        """
        Removes a registered object from listeners

        :param obj_wref: weakref to remove as listener
        :type obj_wref: weakref.ref
        """
        if isinstance(obj_wref, weakref.ref):
            StepListener._subscribers.remove(obj_wref)
        else:
            logger.warn("Trying to unregister non-weakref object")


class PyrepDevice(object):
    """
    PyrepDevice handles communication with CoppeliaSim Vision Sensors via
    Pyrep. Retrieved images are converted to cv2 format and passed to registered
    callback functions.
    """

    @staticmethod
    def autodetect_nicoeyes():
        """
        Returns a tuple containing the detected names of left and right eye
        sensor

        :return: Devicenames as (left, right) tuple (None if not found)
        :rtype: tuple
        """
        eye_sensors = ["left_eye", "right_eye"]
        for i in range(len(eye_sensors)):
            if not VisionSensor.exists(eye_sensors[i]):
                logger.warning(
                    "Could not find vision sensor named '%s'", eye_sensors[i]
                )
                eye_sensors[i] = None
        return tuple(eye_sensors)

    def __init__(
        self,
        sensor_name,
        width=640,
        height=480,
        near_clipping_plane=1e-2,
        far_clipping_plane=10.0,
        view_angle=60.0,
    ):
        """
        Connects to Vision Sensor with given name and updates parameters
        accordingly.

        :param sensor_name: name (or handle) of the vision sensor in the scene
        :type sensor_name: str
        :param width: horizontal camera resolution
        :type width: int
        :param heigth: vertical camera resolution
        :type heigth: int
        :param near_clipping_plane: the minimum distance from which the sensor will be able to detect.
        :type near_clipping_plane: float
        :param far_clipping_plane: the maximum distance from which the sensor will be able to detect.
        :type far_clipping_plane: float
        :param view_angle: field of view of the camera in degree.
        :type view_angle: float
        """
        self._sensor = VisionSensor(sensor_name)
        self.resolution = (width, height)
        self.near_clipping_plane = near_clipping_plane
        self.far_clipping_plane = far_clipping_plane
        self.view_angle = view_angle
        self._callback = []
        self._removed_ids = []
        ref = weakref.ref(self, StepListener.unregister)
        StepListener.register(ref)

    @property
    def sensor_name(self):
        """
        :return: name of the sensor in the scene
        :rtype: str
        """
        return self._sensor.get_name()

    @property
    def resolution(self):
        """
        :return: camera resolution (width, height)
        :rtype: tuple(int, int)
        """
        return tuple(self._sensor.get_resolution())

    @resolution.setter
    def resolution(self, res):
        """
        :param res: camera resolution (width, height)
        :type res: tuple(int, int)
        """
        self._sensor.set_resolution(res)

    @property
    def near_clipping_plane(self):
        """
        :return: the minimum distance from which the sensor will be able to detect.
        :rtype: float
        """
        return self._sensor.get_near_clipping_plane()

    @near_clipping_plane.setter
    def near_clipping_plane(self, val):
        """
        :param val: the minimum distance from which the sensor will be able to detect.
        :type val: float
        """
        self._sensor.set_near_clipping_plane(val)

    @property
    def far_clipping_plane(self):
        """
        :return: the maximum distance from which the sensor will be able to detect.
        :rtype: float
        """
        return self._sensor.get_far_clipping_plane()

    @far_clipping_plane.setter
    def far_clipping_plane(self, val):
        """
        :param val: the maximum distance from which the sensor will be able to detect.
        :type val: float
        """
        self._sensor.set_far_clipping_plane(val)

    @property
    def view_angle(self):
        """
        :return: field of view of the camera in degree.
        :rtype: float
        """
        return self._sensor.get_perspective_angle()

    @view_angle.setter
    def view_angle(self, val):
        """
        :param val: field of view of the camera in degree.
        :type val: float
        """
        self._sensor.set_perspective_angle(val)

    def get_image(self):
        """
        Captures an image for the current simulation step from the
        vision sensor and converts it to cv2 format

        :return: the image
        :rtype: cv2 image
        """
        frame = self._sensor.capture_rgb()
        frame = np.uint8(frame[:, :, [2, 1, 0]] * 255)
        return frame

    def step(self):
        """
        Executes all registered callbacks with the sensor image for the current
        simulation step.
        """
        frame = self.get_image()
        id_start = 0
        # iterate through callbacks while skipping removed ids (if any)
        for id_end in self._removed_ids:
            for id in range(id_start, id_end):
                self._callback[id](frame)
            id_start = id_end
        # iterate through (remaining) callbacks
        for id in range(id_start, len(self._callback)):
            self._callback[id](frame)

    def add_callback(self, function):
        """
        Adds a callback for the event loop

        Whenever step is called, all registered callbacks are executed.

        The callback must take exactly 1 argument through which it receives the
        frame.

        :param function: Function to add as callback
        :type function: function

        :return: id of the added callback
        :rtype: int
        """
        if not (inspect.isfunction(function) or inspect.ismethod(function)):
            logger.error("Trying to add non-function callback")
            raise ValueError("Trying to add non-function callback")
        if len(self._removed_ids) == 0:
            self._callback += [function]
            id = len(self._callback) - 1
        else:
            id = min(self._removed_ids)
            self._callback[id]
            self._removed_ids.remove(id)
        logger.info("Added callback with id %i", id)
        return id

    def remove_callback(self, id):
        """
        Removes callback with given id

        :param id: id of the callback
        :type id: int
        """
        if id in self._removed_ids or id not in range(0, len(self._callback)):
            logger.warning("Callback with id %i does not exist", id)
            return
        if id == len(self._callback) - 1:
            i = id
            for removed in reversed(self._removed_ids):
                if i - 1 != removed:
                    break
                self._removed_ids[:-1]
                i -= 1
            self._callback = self._callback[:i]
        else:
            self._callback[id] = None
            self._removed_ids += [id]
            self._removed_ids.sort()
        logger.info("Removed callback with id %i", id)

    def clean_callbacks(self):
        """
        Removes all saved callbacks
        """
        self._callback.clear()
        self._removed_ids.clear()
        logger.info("Cleared callbacks")
