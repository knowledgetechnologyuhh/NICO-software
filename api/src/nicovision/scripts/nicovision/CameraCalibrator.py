import json
import logging
import threading
import time
from math import ceil
from os.path import abspath, dirname, isfile

import cv2
import numpy as np

import Barrier
import MultiCamRecorder
from NumpyEncoder import NumpyEncoder


def takespread(sequence, num):
    length = float(len(sequence))
    r = []
    for i in range(num):
        r.append(sequence[int(ceil(i * length / num))])
    return r


class CameraCalibrator:
    """
    CameraCalibrator allows to calibrate the cameras with a chessboard
    pattern
    """

    def __init__(
        self,
        devices=[],
        cam_width=640,
        cam_height=480,
        window_width_per_cam=640,
        window_height=480,
        framerate=20,
        zoom=None,
        pan=None,
        tilt=None,
        settings_file=None,
        setting="standard",
        pixel_format="MJPG",
    ):
        """
        Initialises the CameraCalibrator with given devices.

        The devices must be contained in :meth:`get_devices`

        :param devices: Device names (autodetected if empty list)
        :type device: list(str)
        :param cam_width: Width of camera image(s)
        :type cam_width: float
        :param cam_height: Height of camera image(s)
        :type cam_height: float
        :param window_width_per_cam: Adjusted width of the displayed image(s)
        :type window_width_per_cam: float
        :param window_height: Adjusted height of the displayed image(s)
        :type window_height: float
        :param framerate: number of frames per second
        :type framerate: int
        :param value: zoom value between 100 and 800 (overwrites settings)
        :type value: int
        :param value: pan value between -648000 and 648000, step 3600
        (overwrites settings)
        :type value: int
        :param value: tilt value between -648000 and 648000, step 3600
        (overwrites settings)
        :type value: int
        :param setttings_file: the settings file
        :type settings_file: str
        :param setting: name of the setting that should be applied
        :type setting: str
        :param writer_threads: Number of worker threads for image writer
        :type writer_threads: int
        :param pixel_format: fourcc codec
        :type pixel_format: string
        """
        self._logger = logging.getLogger(__name__)

        self._dim = cam_width, cam_height
        self._recorder = MultiCamRecorder.MultiCamRecorder(
            devices,
            cam_width,
            cam_height,
            framerate,
            zoom,
            pan,
            tilt,
            settings_file,
            setting,
            0,
            pixel_format,
        )
        self._deviceIds = self._recorder._deviceIds
        self._current_frames = [np.zeros((cam_height, cam_width, 3), np.uint8)] * len(
            self._deviceIds
        )
        self._stop_event = threading.Event()
        self._display = threading.Thread(
            target=self._display_thread,
            args=(self._stop_event, window_width_per_cam, window_height),
        )
        self._display.daemon = True
        self._display.start()

    def __del__(self):
        cv2.destroyAllWindows()

    def _display_thread(self, stop_event, window_width_per_cam, window_height):
        while not stop_event.is_set():
            resized_frames = [
                cv2.resize(frame, (window_width_per_cam, window_height))
                for frame in self._current_frames
            ]
            cv2.imshow("cameras", cv2.hconcat(resized_frames))
            cv2.waitKey(1)
        cv2.destroyWindow("cameras")

    def _callback(self, rval, frame, id):
        if rval:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            self._rvals[id], corners = cv2.findChessboardCorners(
                gray,
                self._chessboard,
                cv2.CALIB_CB_ADAPTIVE_THRESH
                + cv2.CALIB_CB_FAST_CHECK
                + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )

            try:
                self._chess_detection_barrier.wait()
            except Barrier.BrokenBarrierError:
                return
            # If found, add object points, image points (after refining them)
            if reduce(lambda x, y: x and y, self._rvals) is True:

                corners2 = cv2.cornerSubPix(
                    gray, corners, (3, 3), (-1, -1), self._criteria
                )
                self._imgpoints[id].append(corners2)

                # Draw and display the corners
                self._current_frames[id] = cv2.drawChessboardCorners(
                    frame, self._chessboard, corners2, self._rvals[id]
                )
            else:
                self._current_frames[id] = frame

    def _calibrate_mono(self, id):
        """
        Generates calibration parameters from image points recorded with the
        device at the given index in device ids (NOT the actual device id)

        :param id: position of device in list of deviceIds
                   (NOT the actual device id)
        :type id: int
        """
        if self._imgpoints[id] is None or not self._imgpoints[id]:
            self._logger.warning(
                "Unable to calibrate device "
                + self._deviceIds[id]
                + " - no imagepoints recorded"
            )
            return None

        self._logger.info("Calibrating device {}".format(self._deviceIds[id]))

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self._chessboard[0] * self._chessboard[1], 1, 3), np.float64)
        objp[:, 0, :2] = np.mgrid[
            0 : self._chessboard[0], 0 : self._chessboard[1]
        ].T.reshape(-1, 2)

        objpoints = np.array([objp] * len(self._imgpoints[id]), dtype=np.float64)

        N_OK = len(objpoints)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        ret, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints,
            self._imgpoints[id],
            self._dim,
            K,
            D,
            rvecs,
            tvecs,
            self._calibration_flags,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
        )

        return dict((("K", K), ("D", D)))

    def _calibrate_stereo(self):
        """
        Generates calibration parameters from image points recorded with
        stereo cameras
        """
        if len(self._imgpoints) != 2:
            self._logger.error("Stereo calibration requires exactly 2 devices")
            return None
        if None in self._imgpoints or [] in self._imgpoints:
            self._logger.warning(
                "Unable to calibrate stereo device " + "- no imagepoints recorded"
            )
            return None

        self._logger.info("Calibrating stereo camera")
        N_OK = len(self._imgpoints[0])
        K_left = np.zeros((3, 3))
        D_left = np.zeros((4, 1))
        K_right = np.zeros((3, 3))
        D_right = np.zeros((4, 1))
        R = np.zeros((1, 1, 3), dtype=np.float64)
        T = np.zeros((1, 1, 3), dtype=np.float64)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((self._chessboard[0] * self._chessboard[1], 1, 3), np.float64)
        objp[:, 0, :2] = np.mgrid[
            0 : self._chessboard[0], 0 : self._chessboard[1]
        ].T.reshape(-1, 2)

        objpoints = np.array([objp] * len(self._imgpoints[0]), dtype=np.float64)
        imgpoints_left = np.asarray(self._imgpoints[0], dtype=np.float64)
        imgpoints_right = np.asarray(self._imgpoints[1], dtype=np.float64)

        objpoints = np.reshape(
            objpoints, (N_OK, 1, self._chessboard[0] * self._chessboard[1], 3)
        )
        imgpoints_left = np.reshape(
            imgpoints_left, (N_OK, 1, self._chessboard[0] * self._chessboard[1], 2)
        )
        imgpoints_right = np.reshape(
            imgpoints_right, (N_OK, 1, self._chessboard[0] * self._chessboard[1], 2)
        )

        ret, K_left, D_left, K_right, D_right, R, T = cv2.fisheye.stereoCalibrate(
            objpoints,
            imgpoints_left,
            imgpoints_right,
            K_left,
            D_left,
            K_right,
            D_right,
            self._dim,
            R,
            T,
            self._calibration_flags,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 12, 0)
            # 30, 1e-6)
        )
        R_left, R_right, P_left, P_right, Q = cv2.fisheye.stereoRectify(
            K_left, D_left, K_right, D_right, self._dim, R, T, cv2.CALIB_ZERO_DISPARITY
        )

        return dict(
            (
                ("K_left", K_left),
                ("D_left", D_left),
                ("K_right", K_right),
                ("D_right", D_right),
                ("R", R),
                ("T", T),
                ("R_left", R_left),
                ("R_right", R_right),
                ("P_left", P_left),
                ("P_right", P_right),
                ("Q", Q),
            )
        )

    def start_calibration(
        self,
        chessboard=(11, 8),
        duration=30,
        number_of_samples=100,
        stereo=True,
        calibration_file=(
            dirname(abspath(__file__))
            + "/../../../../../json/"
            + "nico_vision_calibration_params.json"
        ),
        overwrite=False,
        term_criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1),
        calibration_flags=(
            cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
            + cv2.fisheye.CALIB_CHECK_COND
            + cv2.fisheye.CALIB_FIX_SKEW
        ),
    ):
        """
        Records images for given duration to calibrate cameras.

        Calibration related code was partially taken from:
        https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-333b05afa0b0
        and
        https://github.com/sourishg/fisheye-stereo-calibration/blob/master/calibrate.cpp
        (stereo term_criteria and fov_scale)

        :param chessboard: Dimensions of the chessboard pattern (inner corners)
        :type chessboard: tuple(int)
        :param duration: Duration of the recording
        :type duration: int
        :param number_of_samples: Subset of images used for calibration
                                  (to reduce computing time)
        :type number_of_samples: int
        :param stereo: Whether cameras should be calibrated individually or as
                       stereo cameras
        :type overwrite: bool
        :param calibration_file: json file path to save calibration parameters
        :type calibration_file: str
        :param overwrite: Whether preexisting parameters in the file should be
                          overwritten
        :type overwrite: bool
        :param term_criteria: cv2 term_criteria for calibration
        :type term_criteria: list
        :param calibration_flags: cv2 calibration_flags
        :type calibration_flags: list
        """
        self._chessboard = chessboard
        self._criteria = term_criteria
        self._calibration_flags = calibration_flags
        # 2d points in image plane.
        self._imgpoints = [[]] * len(self._deviceIds)
        self._rvals = [False] * len(self._deviceIds)
        self._chess_detection_barrier = Barrier.Barrier(len(self._deviceIds))
        devicenames = MultiCamRecorder.get_devices()
        devicenames = map(lambda i: devicenames[i], self._deviceIds)
        zoom = self._device.get_zoom()
        # load preexisting calibrations from file
        if isfile(calibration_file):
            with open(calibration_file, "r") as existing_file:
                existing_calibration = json.load(existing_file)
                if stereo and str(devicenames) not in existing_calibration["stereo"]:
                    existing_calibration["stereo"][str(devicenames)] = {}
                elif (
                    stereo
                    and str(self._dim)
                    in existing_calibration["stereo"][str(devicenames)]
                ):
                    existing_calibration["stereo"][str(devicenames)][
                        str(self._dim)
                    ] = {}
                elif not stereo:
                    for name in devicenames:
                        if name not in existing_calibration["mono"]:
                            existing_calibration["mono"][name] = {}
                        elif str(self._dim) not in existing_calibration["mono"][name]:
                            existing_calibration["mono"][name][str(self._dim)] = {}
        else:
            existing_calibration = {
                "stereo": {str(devicenames): {str(self._dim): {}}},
                "mono": dict(
                    zip(devicenames, [{str(self._dim): {}} for _ in devicenames])
                ),
            }
        # abort if calibration for device and dim already exists and overwrite
        # not enabled
        if not overwrite:
            if (
                stereo
                and str(zoom)
                in existing_calibration["stereo"][str(devicenames)][str(self._dim)]
            ):
                self._logger.warning(
                    (
                        "Calibration aborted - Overwrite not "
                        + "enabled and setting for devices {} "
                        + "and dimension {} already exists in {}"
                    ).format(devicenames, self._dim, calibration_file)
                )
                return
            elif not stereo:
                for i in range(len(self._deviceIds)):
                    if (
                        str(zoom[i])
                        in existing_calibration["mono"][str(devicenames[i])][
                            str(self._dim)
                        ]
                    ):
                        self._logger.warning(
                            (
                                "Calibration aborted - Overwrite "
                                + "not enabled and setting for "
                                + "device {} and dimension {} "
                                + "already exists in {}"
                            ).format(devicenames[i], self._dim, calibration_file)
                        )
                        return
        # start recording
        self._logger.info("Start recording images for calibration")
        self._recorder.add_callback(self._callback)
        self._recorder._open = True
        time.sleep(duration)
        self._chess_detection_barrier.abort()
        self._recorder.stop_recording()
        self._stop_event.set()
        self._display.join()
        time.sleep(1)
        self._logger.info("Recording finished - preparing for calibration")
        # reduce recorded image points to number of samples
        new_length = min(
            number_of_samples, len(self._imgpoints[0]), len(self._imgpoints[1])
        )
        self._imgpoints = map(lambda x: takespread(x, new_length), self._imgpoints)
        # start calibration
        if stereo:
            calib_params = self._calibrate_stereo()
            if calib_params:
                existing_calibration["stereo"][str(devicenames)][str(self._dim)][
                    str(zoom)
                ] = calib_params
        else:
            for i in range(len(self._deviceIds)):
                calib_params = self._calibrate_mono(i)
                if calib_params:
                    existing_calibration["mono"][devicenames[i]][str(self._dim)][
                        str(zoom[i])
                    ] = calib_params
        # save results
        self._logger.info("Calibration finished - saving results")
        self._logger.debug("Saving calibration {}".format(existing_calibration))
        with open(calibration_file, "w") as outfile:
            json.dump(existing_calibration, outfile, cls=NumpyEncoder)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    calibrator = CameraCalibrator(
        cam_width=1920,
        cam_height=1080,
        framerate=10,
        window_width_per_cam=768,
        window_height=432,
        zoom=200,
    )
    # to reduce complexity of calibration, 100 (number_of_samples) evenly
    # distributed samples will be taken from the total amount of recorded
    # frames
    calibrator.start_calibration(
        chessboard=(6, 7),
        stereo=True,
        duration=90,
        number_of_samples=90,
        overwrite=True,
    )
