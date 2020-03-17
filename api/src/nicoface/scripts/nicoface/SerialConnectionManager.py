import logging
import serial
import serial.tools.list_ports
import threading
import time


class SerialDevice(object):

    """This class allows multiple objects to share the same serial connection."""

    _serial_connections = {}  # DO NOT OVERRIDE!!!
    _logger = logging.getLogger(__name__)

    @staticmethod
    def get_devices_by_manufacturer(manufacturer):
        ports = serial.tools.list_ports.comports()
        return [
            p.device for p in ports if p.manufacturer and manufacturer in p.manufacturer
        ]

    def __init__(self, device, baudrate, timeout):
        self._device = device
        self._logger.info("Connecting to %s", device)
        if device in self._serial_connections:
            self._logger.debug("Using existing connection to %s", device)
            ser, connected, mutex = self._serial_connections[device]
            mutex.acquire()
            if ser.baudrate != baudrate or ser.timeout != timeout:
                mutex.release()
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
                mutex.release()
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
                threading.Semaphore(),
            ]
            time.sleep(2)  # delay to wait for arduino to reset

    def send(self, message):
        ser, connected, mutex = self._serial_connections[self._device]
        mutex.acquire()
        self._logger.debug("Sending '%s' to '%s'", message, self._device)
        ser.write(message.encode("utf-8"))
        response = ser.readline()
        while ser.in_waiting:
            response += ser.readline()
        self._logger.debug("Received %s from '%s'", repr(response), self._device)
        mutex.release()
        return response

    def close(self):
        if self._device in self._serial_connections:
            self._logger.debug("Detaching %s from device %s", str(self), self._device)
            ser, connected, mutex = self._serial_connections[self._device]
            mutex.acquire()
            connected.discard(str(self))
            if len(connected) == 0:
                self._logger.info("Closing device %s", self._device)
                ser.close()
                self._serial_connections.pop(self._device)
            mutex.release()

    def reset(self):
        if self._device in self._serial_connections:
            ser, connected, mutex = self._serial_connections[self._device]
            mutex.acquire()
            self._logger.debug("Resetting %s", str(self), self._device)
            ser.close()
            ser.open()
            time.sleep(1)
            mutex.release()

    def flushInput(self):
        if self._device in self._serial_connections:
            ser, connected, mutex = self._serial_connections[self._device]
            mutex.acquire()
            self._logger.debug("Flushing %s", str(self), self._device)
            ser.flushInput()
            mutex.release()

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
