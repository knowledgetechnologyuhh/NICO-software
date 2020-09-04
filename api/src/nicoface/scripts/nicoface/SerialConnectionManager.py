import logging
import posix_ipc
import serial
import serial.tools.list_ports
import time


class SerialDevice(object):

    """This class allows multiple objects to share the same serial connection."""

    _serial_connections = {}  # DO NOT OVERRIDE!!!
    _logger = logging.getLogger(__name__)

    @staticmethod
    def get_devices_by_manufacturer(manufacturer):
        """
        Returns all devices for a given manufacturer

        :param manufacturer: (partial) name of the manufacturer
        :type filename: str
        :return: list of devices
        :rtype: list(str)
        """
        ports = serial.tools.list_ports.comports()
        return [
            p.device for p in ports if p.manufacturer and manufacturer in p.manufacturer
        ]

    def __init__(self, device, baudrate, timeout):
        """
        Connects to a serial device or reuses an existing connection if possible

        :param device: Device name to connect to
        :type device: str
        :param baudrate: Baud rate of the connection
        :type baudrate: int
        :param timeout: Read timeout value
        :type timeout: float
        """
        self._device = device
        self._logger.info("Connecting to %s", device)
        if device in self._serial_connections:
            self._logger.debug("Using existing connection to %s", device)
            ser, connected, mutex = self._serial_connections[device]
            with mutex:
                if ser.baudrate != baudrate or ser.timeout != timeout:
                    raise serial.SerialException(
                        (
                            "Failed to open %s with baudrate %i and timeout %f - "
                            "Existing connection with baudrate %i and timeout %f"
                        ),
                        device,
                        baudrate,
                        timeout,
                        ser.baudrate,
                        ser.timeout,
                    )
                else:
                    connected.add(str(self))
        else:
            self._logger.debug(
                "Opened new connection to %s with baudrate %i and timeout %s",
                device,
                baudrate,
                timeout,
            )
            self._serial_connections[device] = [
                serial.Serial(device, baudrate, timeout=timeout),
                {str(self)},
                posix_ipc.Semaphore(
                    device.split("/")[-1], posix_ipc.O_CREAT, initial_value=1
                ),
            ]
            time.sleep(2)  # delay to wait for arduino to reset

    def send(self, message):
        """
        Sends a message to the device and returns the received response

        :param message: message to send to the device
        :type message: str
        :return: received response
        :rtype: str
        """
        ser, connected, mutex = self._serial_connections[self._device]
        with mutex:
            self._logger.debug("Sending '%s' to '%s'", message, self._device)
            ser.write(message.encode("utf-8"))
            response = ser.readline()
            while ser.in_waiting:
                response += ser.readline()
            self._logger.debug("Received %s from '%s'", repr(response), self._device)
        return response

    def close(self):
        """
        Detaches from the device, closes connection if no other object
        is attached
        """
        if self._device in self._serial_connections:
            self._logger.debug("Detaching %s from device %s", str(self), self._device)
            ser, connected, mutex = self._serial_connections[self._device]
            with mutex:
                connected.discard(str(self))
                if len(connected) == 0:
                    self._logger.info("Closing device %s", self._device)
                    ser.close()
                    self._serial_connections.pop(self._device)
            mutex.unlink()

    def reset(self):
        """
        Closes and reopens the connection to the serial device
        """
        if self._device in self._serial_connections:
            ser, connected, mutex = self._serial_connections[self._device]
            with mutex:
                self._logger.debug("Resetting connection to %s", self._device)
                ser.close()
                ser.open()
                time.sleep(1)

    def flushInput(self):
        """
        Flush input buffer of the connected serial device
        """
        if self._device in self._serial_connections:
            ser, connected, mutex = self._serial_connections[self._device]
            with mutex:
                self._logger.debug("Flushing %s", self._device)
                ser.reset_input_buffer()

    def __del__(self):
        self.close()


if __name__ == "__main__":

    logging.basicConfig(level=logging.DEBUG)

    a = SerialDevice("/dev/ttyACM0", 115200, 0.016)
    print("happiness")
    response = a.send("happiness")
    print(response)
    time.sleep(0.3)
    print("anger")
    response = a.send("anger")
    print(response)
    time.sleep(0.3)
    print("unknown")
    response = a.send("unknown")
    print(response)
    time.sleep(0.3)
    print("sadness")
    response = a.send("sadness")
    print(response)
    time.sleep(0.3)
    print("neutral")
    response = a.send("neutral")
    print(response)
