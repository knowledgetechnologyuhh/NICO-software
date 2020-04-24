import datetime
import logging
import threading
import time
import weakref

import serial
import serial.tools.list_ports

import nicotouch._nicotouch_internal.optoforce as optoforce_driver

# 'Counts/N' from sensitivity report
scales = {
    "ONR0A003": [
        [230.34, 230.34, 230.34],  # DSE0A174 (ring)
        [186.19, 186.19, 186.19],  # DSE0A173 (index)
        [189.20, 189.20, 189.20],  # DSE0A184 (thumb)
        [1, 1, 1],
    ]  # unused 4th slot
}

# labels for fingers
keys = {"ONR0A003": ["ring", "index", "thumb", None]}


def _cache_thread(object, frequency):
    """
    Thread that caches sensor values at defined frequency. Only pass weakrefs!
    If object is not a weakref, it may introduce memory leaks!

    """
    while object is not None:
        time_before = time.time()
        object().update_cache()
        run_time = time.time() - time_before
        time.sleep(max((1.0 / frequency) - run_time, 0))


class OptoforceMultichannel(object):
    """
    OptoforceMultichannel allows access to multiple OptoForce sensors Connected
    to the same device
    """

    def _scan_ports(self, ser_number=None):
        """
        Scans serial ports for OptoForce device with given serial number and
        returns the connection.
        Returns the first sensor found if no serial number is given.

        :param ser_number: Serial number of the sensor (optional)
        :type ser_number: str
        :return: Connected OptoforceDriver with specified serial number
                 (or any if ser_number is None)
        :rtype: _nicotouch_internal.optoforce.OptoforceDriver
        """
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "OptoForce" in p.description:
                logging.info(
                    ("Connecting to OptoForce sensor on port {}").format(p.device)
                )
                try:
                    driver = optoforce_driver.OptoforceDriver(
                        p.device, "m-ch/3-axis", self._scale
                    )

                    driver.request_serial_number()
                    while driver._serial.is_open:
                        try:
                            data = driver.read()

                            if isinstance(data, optoforce_driver.OptoforceSerialNumber):
                                if ser_number is None or ser_number == str(data):
                                    logging.info(
                                        (
                                            "Successfully connected to "
                                            + "OptoForce sensor {} on port {}"
                                        ).format(str(data), p.device)
                                    )
                                    self._ser_number = str(data)
                                    return driver
                                logging.info(
                                    (
                                        "OptoForce sensor on port {} skipped - "
                                        + "serial number {} not matching {}"
                                    ).format(p.device, ser_number, str(data))
                                )
                                driver._serial.close()
                                del driver

                        except optoforce_driver.OptoforceError:
                            pass

                except serial.SerialException as e:
                    logging.warning(
                        (
                            "Connection to OptoForce sensor port {} failed due "
                            + "to {}"
                        ).format(p.device, e)
                    )

        if ser_number is None:
            logging.fatal("No OptoForce sensor found")
        else:
            logging.fatal(
                ("No matching OptoForce sensor found for serial number {}").format(
                    ser_number
                )
            )
        return None

    def __init__(self, ser_number=None, cache_frequency=None):
        """
        Connects to optoforce sensor with given serial number (autodetects if
        None) and starts caching if a frequency is given. (otherwise a new value
        is fetched on method call)

        :param ser_number: Serial number of the sensor (optional)
        :type ser_number: str
        :param ser_number: Cache frequency (optional)
        :type ser_number: int
        """

        self._logger = logging.getLogger(__name__)
        self._scale = [1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1]
        self._driver = self._scan_ports(ser_number)

        if self._ser_number in scales:
            self._scale = scales[self._ser_number]
        else:
            self._logger.warning(
                (
                    "Missing 'Counts/N' entry from sensitivity report for {}"
                    + " - conversion to Newton won't be possible"
                ).format(self._ser_number)
            )

        if self._ser_number in keys:
            self._keys = keys[self._ser_number]
        else:
            self._keys = ["0", "1", "2", "3"]
            self._logger.warning(
                (
                    "No name keys defined for {} - using generic keys {} " + "instead"
                ).format(self._ser_number, self._keys)
            )

        self._cached_data = None
        self._cached_mode = False
        self._mutex = threading.Semaphore()

        # Run in cached mode, Start the worker thread to get the data
        if cache_frequency:
            self._cached_mode = True
            worker = threading.Thread(
                target=_cache_thread, args=(weakref.ref(self), cache_frequency)
            )
            worker.daemon = True
            worker.start()

        time.sleep(0.2)

    def update_cache(self):
        """
        Collects new sensor values from the device.
        """
        while True:
            self._driver.flush()
            data = self._driver.read()
            if isinstance(data, optoforce_driver.OptoforceData):
                self._mutex.acquire()
                self._last_update = datetime.datetime.now().isoformat()
                self._cached_data = data
                self._mutex.release()
                break

    def get_sensor_values_raw(self):
        """
        Returns latest raw sensor readings for each channel

        :return: "time", "count", "status" and raw x,y,z "forces" for each
                 channel (see global OptoforceMultichannel.keys for channel
                 keys)
        :rtype: dict
        """
        ret = {}
        if not self._cached_mode:
            self.update_cache()

        self._mutex.acquire()
        ret["forces"] = dict(zip(self._keys, self._cached_data.force))
        ret["time"] = self._last_update
        ret["count"] = self._cached_data.count
        ret["status"] = self._cached_data.status
        self._mutex.release()

        return ret

    def get_sensor_values(self):
        """
        Returns latest newton sensor readings for each channel

        :return: "time", "count", "status" and newton x,y,z "forces" for each
                 channel (see global OptoforceMultichannel.keys for channel
                 keys)
        :rtype: dict
        """
        raw = self.get_sensor_values_raw()
        # convert forces for each sensor
        for key in self._keys:
            # elementwise division
            raw["forces"][key] = map(
                lambda val, scale: val / scale,
                raw["forces"][key],
                self._scale[self._keys.index(key)],
            )
        return raw
