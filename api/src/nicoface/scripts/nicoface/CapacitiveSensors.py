import logging
from serial import SerialException
from SerialConnectionManager import SerialDevice


class CapacitiveSensors(object):
    """
    CapacitiveSensors handles communication with the capacitive sensors
    inside the NICO head.

    Based on contributions by Seed Robotics (www.seedorobotics.com)
    """

    def __init__(self, devicename=None):
        """
        Connect to serial device with the given name
        (autodetected if left out).

        :param devicename: Name of the serial device
        :type devicename: str
        """

        self._logger = logging.getLogger(__name__)

        baudrate = 115200
        timeout = 0.016

        if devicename is None:
            self._scan_ports(baudrate, timeout)
        else:
            # Establish the connection on a specific port
            self.ser = SerialDevice(devicename, baudrate, timeout=timeout)

    def _scan_ports(self, baudrate, timeout):
        """
        Automatically detects and establishes a connection with the
        FaceExpression Arduino

        :param baudrate: Baud rate of the connection
        :type baudrate: int
        :param timeout: Read timeout value
        :type timeout: float
        """
        devices = SerialDevice.get_devices_by_manufacturer("duino")
        for device in devices:
            self._logger.info("Connecting to Arduino on port %s", device)
            try:
                self.ser = SerialDevice(device, baudrate, timeout=timeout)

                self._logger.debug("Trying to send neutral face expression")
                if self.getCapacitiveReadings():
                    self._logger.info(
                        "Successfully connected to FaceExpression "
                        + "device on port %s",
                        device,
                    )
                    return
                self.ser.close()

            except SerialException as e:
                self._logger.warning(
                    ("Connection to Arduino on port %s failed due to %s"), device, e
                )

        self._logger.fatal("No FaceExpression Arduino device found")
        self.ser = None
        exit(1)

    def getCapacitiveReadings(self):
        """
        Returns a list with the capacitive readings. Use the /len()/ python
        operator to determine the size of the list.

        An empty object is returned if there is a communication failure or the
        feature is not supported on the Arduino sketch running in the head.

        :return: capacitive readings
        :rtype: list
        """
        self._logger.info("Querying capacitive readings using command 'cprr'")
        # clear all buffered data to ensure we only get the data for our command
        self.ser.flushInput()
        response = self.ser.send("caprr")

        # Format of the Serial measage, matched to the Arduino skecth:
        # Byte 0=Nr capacitive pads
        # Byte 1=size of each reading (in nr of bytes). This to account for systems where an INT can be 2 or 4 bytes...
        # the values in a sequence
        # Last byte=checksum is calculated in the same way as Dynamixel 1: add all data bytes and then negate the result

        out_readings = []
        barray_readings = bytearray(response)

        if not self.validResponse(barray_readings):
            return

        # reconstruct the values from the bytes sent
        nr_pads = barray_readings[0]
        data_size = barray_readings[1]

        for b in range(0, nr_pads):
            accum_value = 0
            for c in range(0, data_size):
                curr_value = barray_readings[2 + b * data_size + c]
                # "2 +" bc the first 2 bytes are status bytes (data len, nr pads)
                accum_value += curr_value * (
                    256 ** c
                )  # note the '**' is the "power" operator (same as math.pow)
            out_readings.append(accum_value)

        return out_readings

    def recallibrateCapacitivePads(self):
        """
        Recallibrates the zero/baseline reading on the capacitive pads.

        By default pads are callibrated on power up of the head board. Use this
        function only if you find significant deviations during operation.
        """
        self._logger.info("Reacallibrating capacitive pads using command 'capca'")
        # clear all buffered data to ensure we only get the data for our command
        self.ser.flushInput()
        response = self.ser.send("capca")

        if response == b"":
            self._logger.warning("No response to Capacitive Query")
            return
        else:
            self._logger.info("Re callibration result: {}".format(response))

    def validResponse(self, barray_readings):
        """
        Checks validity of received response

        :param barray_readings: response to verify
        :type barray_readings: bytearray

        :return: validity of the response
        :rtype: bool
        """
        if len(barray_readings) < 3:
            self._logger.warning(
                "Invalid response to Capacitive Query. Less than 3 bytes"
            )
            return False

        nr_pads = barray_readings[0]
        data_size = barray_readings[1]

        if len(barray_readings) != nr_pads * data_size + 3:
            self._logger.warning(
                "Invalid response to Capacitive Query. nr_pads=%i and data_size=%i don't match barray_len=%i",
                nr_pads,
                data_size,
                len(barray_readings),
            )
            return False
        checksum = barray_readings[-1]
        bytesum = sum(barray_readings[2:-1]) % 256

        if (bytesum ^ checksum) != 0xFF:
            self._logger.warning(
                "Invalid response to Capacitive Query. Checksum failure."
            )
            return False

        return True
