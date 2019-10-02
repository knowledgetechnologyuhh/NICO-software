import codecs
import datetime
import logging
import threading
import time

import serial
import serial.tools.list_ports

from ._nicotouch_internal import optoforce as optoforce_driver


"""
Created on Thu Jun 23 15:32:38 2016

sensor access by Moaaz Maamoon M. Ali

adapted to NICO api Erik Strahl and Connor Gaede

"""


class optoforce():

    def _scan_ports(self, ser_number=None):
        """
        Scans serial ports for OptoForce device with given serial number and
        returns the port. Returns the first sensor found if no serial number is
        given.

        :param ser_number: Serial number of the sensor (optional)
        :type ser_number: str
        :return: Port of OptoForce sensor with specified serial number (or
                 first one found if ser_number is None)
        :rtype: str
        """
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "OptoForce" in p.description:
                self._logger.info(
                    (
                        "Connecting to OptoForce sensor on port {}"
                    ).format(p.device))
                try:
                    driver = optoforce_driver.OptoforceDriver(
                        p.device, "s-ch/3-axis", [[1, 1, 1]])

                    driver.request_serial_number()
                    while driver._serial.is_open:
                        try:
                            data = driver.read()

                            if isinstance(
                                    data,
                                    optoforce_driver.OptoforceSerialNumber
                            ):
                                if(ser_number is None
                                   or ser_number == str(data)):
                                    self._logger.info(
                                        ("Successfully connected to " +
                                         "OptoForce sensor {} on port {}"
                                         ).format(
                                            str(data), p.device))
                                    del driver
                                    self._ser_number = str(data)
                                    return p.device
                                self._logger.info(
                                    ("OptoForce sensor on port {} skipped - " +
                                     "serial number {} not matching {}"
                                     ).format(
                                        p.device, ser_number, str(data)))
                                driver._serial.close()

                        except optoforce_driver.OptoforceError:
                            pass

                except serial.SerialException as e:
                    self._logger.warning(("Connection to OptoForce sensor " +
                                          "port {} failed due to {}"
                                          ).format(p.device, e))

        if ser_number is None:
            self._logger.fatal("No OptoForce sensor found")
        else:
            self._logger.fatal("No matching OptoForce sensor found for " +
                               "serial number {}".format(ser_number))
        exit(1)

    def get_sensor_values_m(self):
        sensorValues = self.ser.read(16)
        xAxis = int((sensorValues[8] + sensorValues[9]).encode('hex'), 16)
        if xAxis > 0x7FFF:
            xAxis -= 0x10000
        # xAxis -= 49
        yAxis = int((sensorValues[10] + sensorValues[11]).encode('hex'), 16)
        if yAxis > 0x7FFF:
            yAxis -= 0x10000
        # yAxis -=  42
        zAxis = int((sensorValues[12] + sensorValues[13]).encode('hex'), 16)
        if zAxis > 0x7FFF:
            zAxis -= 0x10000
            # zAxis += 255
        return(xAxis, yAxis, zAxis)
        # print ('X Axis: ', xAxis, ' Y Axis: ', yAxis,' Z Axis: ',zAxis )
        # i += 1

    def get_sensor_values_hex(self):
        if self.cache_frequency and self.cached_sensor_array is not None:
            seq = self.cached_sensor_array
        else:
            seq = self.get_sensor_array()
        seq = self.get_sensor_array()
        x = seq[8] + seq[9]
        y = seq[10] + seq[11]
        z = seq[12] + seq[13]
        return (x, y, z)

    def get_sensor_values_from_array(self, seq):
        # maxint signed
        maxint_s = 0x7FFF
        # maxint_unsigned_plus_one
        maxint_u_po = 0x10000
        x = int(seq[8] + seq[9], 16)
        if x > maxint_s:
            x = -maxint_u_po + x
        y = int(seq[10] + seq[11], 16)
        if y > maxint_s:
            y = -maxint_u_po + y
        z = int(seq[12] + seq[13], 16)
        if z > maxint_s:
            z = -maxint_u_po + z
        return(x, y, z)

    def get_sensor_values_raw(self):

        if self.cache_frequency and self.cached_sensor_array is not None:
            seq = self.cached_sensor_array
        else:
            seq = self.get_sensor_array()

        (x, y, z) = self.get_sensor_values_from_array(seq)
        return (x, y, z)

    def get_sensor_values(self):
        x, y, z = self.get_sensor_values_raw()
        quot = self.dev_counts / self.dev_nom_capacity * 1.0
        xn = x / quot
        yn = y / quot
        zn = z / quot
        return (xn, yn, zn)

    def get_sensor_all(self):
        # return (time of sensor reading,reading counter, x,y,z, checksum)
        if self.cache_frequency and self.cached_sensor_array is not None:
            seq = self.cached_sensor_array
        else:
            seq = self.get_sensor_array()
        (x, y, z) = self.get_sensor_values_from_array(seq)
        count = int(seq[4] + seq[5], 16)
        # leave status in HEX
        status = seq[6] + seq[7]
        checksum = int(seq[14] + seq[15], 16)
        return (self.last_reading_time, count, status, x, y, z, checksum)

    def get_sensor_array(self):

        # delete the comm buffers
        self.ser.flushInput()
        self.ser.flushOutput()
        '''
        # approach for taking the start sequence of the protocol,
        # but this is not needed here
        seq=["aa"]
        one_byte = self.ser.read().encode('hex')
        while (one_byte!="aa"):
            one_byte = self.ser.read().encode('hex')
        #sensor_string = self.ser.read(16)
        one_byte = self.ser.read().encode('hex')
        #for c in self.ser.read()
        while (one_byte != "aa"):
            print one_byte
            seq.append(one_byte)  # convert from ANSII
            joined_seq = ''.join(str(v) for v in seq)# Make a string from array
            one_byte = self.ser.read().encode('hex')
        return joined_seq
        '''
        # read sequence and convert from ANSII
        seq = codecs.encode(self.ser.read(16), "hex")
        self.last_reading_time = datetime.datetime.now().isoformat()
        return [seq[i:i+2] for i in range(0, len(seq), 2)] # split into bytes

    def get_sensor_string(self):
        seq = self.get_sensor_array()
        joined_seq = ''.join(str(v)
                             for v in seq)  # Make a string from array
        return joined_seq

    def _worker_thread(self):
        while True:
                # we need better accurate frequency
                # so substract the running time from the delay
            time_before = time.time()
            self.cached_sensor_array = self.get_sensor_array()
            run_time = time.time() - time_before
            time.sleep((1.0 / self.cache_frequency) - run_time)

    # Can be run cached in memory or non cached
    # non cahced will get live data from the USB-Serial
    # and will take some time
    # Start the cached mode by giving a cache frequency
    def __init__(self, ser_number=None, cache_frequency=None):
        self._logger = logging.getLogger(__name__)

        self.ser = serial.Serial(port=self._scan_ports(ser_number),
                                 baudrate=1000000,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS
                                 )

        # Capacitys and Counts (measured at optoforce) for our sensors
        self.dev_nom_capacity = 10
        self.dev_counts = None
        self.last_reading_time = None

        if (self._ser_number == "DSE0A125"):
            self.dev_counts = 4014
        elif (self._ser_number == "DSE0A093"):
            self.dev_counts = 4263
        else:
            self._logger.warning(
                (
                    "Missing 'Counts' entry from sensitivity report for {} -" +
                    " conversion to Newton won't be possible"
                ).format(self._ser_number))

        self.cache_frequency = cache_frequency

        self.cached_sensor_array = None

        # Run in cached mode, Start the worker thread to get the data
        if self.cache_frequency:
            worker = threading.Thread(target=self._worker_thread)
            worker.daemon = True
            worker.start()

        time.sleep(0.2)

        # if self.ser.isOpen():
        #    print 'Sensor detected and port opened successfully'
        # writtenMessage = bytearray([170, 0, 50, 3, 10, 4, 255, 1, 236])


if __name__ == "__main__":

    import sys

    import argparse

    # examples
    # Get one raw sample from the sensor at /dev/ttyACM0 and the serial-number
    # of the sensor DSE0A125
    # python optoforcesensors.py raw --device /dev/ttyACM0 --serial DSE0A125
    # Move to position stored in move file
    # python Mover.py pp --file mov_take_something_with_left_arm.csv
    #                    --subset subset_left_arm_and_head.csv --speed 0.1
    # Record move file (trajectory)

    parser = argparse.ArgumentParser()
    parser.add_argument("command",
                        help="One of the commands raw (get raw sensor " +
                        "values), newton (get sensor values in Newton)," +
                        " string ( get the whole message as " +
                        "hex-string-representation) all (get all data " +
                        "(time,sample counter, status,x,y,z,checksum)), " +
                        "csv (all data in csv format to store this right away)"
                        )
    # fj freeze joints as they are by torquing it. You subset to freeze only a
    # subset of the joints.
    parser.add_argument('--serial', nargs='?', default=None,
                        help="serial number of the sensors device")
    parser.add_argument('--cont', action="store_true", default=False,
                        help="do not stop after one reading")
    # parser.add_argument('--stiffoff', action="store_true", default=False,
    #                    help="sets the stiffness to off after movement")
    args = parser.parse_args()
    # print args

    command = args.command

    # optsens = optoforce(args.serial,cache_frequency=30)
    optsens = optoforce(ser_number=None, cache_frequency=30)

    oneTime = True
    while args.cont or oneTime:

        oneTime = False

        # optsens.get_sensor_values_mh()

        if command == "raw":
            (x, y, z) = optsens.get_sensor_values_raw()
            print("x ,y, z : " + str(x) + "," + str(y) + "," + str(z))

        elif command == "newton":
            (x, y, z) = optsens.get_sensor_values()
            print("x ,y, z (in Newton) : " +
                  str(x) + "," + str(y) + "," + str(z))

        elif command == "string":
            print(optsens.get_sensor_string())

        elif command == "all":
            (stime, counter, status, x, y, z, checksum) = \
                optsens.get_sensor_all()
            print("time, counter, status,x, y, z,checksum " +
                  str((stime, counter, status, x, y, z, checksum)))

        elif command == "csv":
            (stime, counter, status, x, y, z, checksum) = \
                optsens.get_sensor_all()
            print(str(stime) + "," + str(counter) + "," + str(status) + "," +
                  str(x) + "," + str(y) + "," + str(z) + "," + str(checksum))

        time.sleep(0.03)
