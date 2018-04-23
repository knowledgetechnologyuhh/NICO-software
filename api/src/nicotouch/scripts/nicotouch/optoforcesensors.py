import serial
import time
import numpy as np
import sys
import datetime

"""
Created on Thu Jun 23 15:32:38 2016

sensor access by Moaaz Maamoon M. Ali

adapted to NICO api Erik Strahl

"""


class optoforce:


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
        return(xAxis,yAxis,zAxis)
        #print ('X Axis: ', xAxis, ' Y Axis: ', yAxis,' Z Axis: ',zAxis )
            # i += 1

    def get_sensor_values_hex(self):
        seq = self.get_sensor_array()
        x=seq[8]+seq[9]
        y = seq[10] + seq[11]
        z = seq[12] + seq[13]
        return (x,y,z)

    def get_sensor_values_from_array(self,seq):
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
        return(x,y,z)

    def get_sensor_values_raw(self):

        seq = self.get_sensor_array()
        (x,y,z)=self.get_sensor_values_from_array(seq)
        return (x,y,z)

    def get_sensor_values(self):
        x,y,z = self.get_sensor_values_raw()
        quot=self.dev_counts*self.dev_nom_capacity*1.0
        xn=x/quot
        yn=y/quot
        zn=z/quot
        return (xn,yn,zn)

    def get_sensor_all(self):
        #return (time of sensor reading,reading counter, x,y,z, checksum)
        seq = self.get_sensor_array()
        (x,y,z)=self.get_sensor_values_from_array(seq)
        count = int(seq[4] + seq[5], 16)
        #leave status in HEX
        status = seq[6] + seq[7]
        checksum = int(seq[14] + seq[15], 16)
        return (self.last_reading_time,count,status,x,y,z,checksum)

    def get_sensor_array(self):

        #delete the comm buffers
        self.ser.flushInput()
        self.ser.flushOutput()
        seq = []
        '''
        # approach for taking the start sequence of the protocol, but this is not needed here
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
            joined_seq = ''.join(str(v) for v in seq)  # Make a string from array
            one_byte = self.ser.read().encode('hex')
        return joined_seq
        '''
        one_sequence = self.ser.read(16)
        self.last_reading_time=datetime.datetime.now().isoformat()
        for c in one_sequence:
            seq.append(c.encode('hex'))  # convert from ANSII

        return seq

    def get_sensor_string(self):
        seq = self.get_sensor_array()
        joined_seq = ''.join(str(v) for v in seq)  # Make a string from array
        return joined_seq

    def __init__(self,port,ser_number):

        #print "Using port: " + port
        self.ser = serial.Serial(port=port,
                            baudrate=1000000,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS
                            )

        #Capacitys and Counts (measured at optoforce) for our sensors
        self.dev_nom_capacity=10
        self.dev_counts=None
        self.last_reading_time=None

        if (ser_number=="DSE0A125"):
            self.dev_counts=4014

        if (ser_number=="DSE0A093"):
            self.dev_counts=4263

        #if self.ser.isOpen():
        #    print 'Sensor detected and port opened successfully'
        #writtenMessage = bytearray([170, 0, 50, 3, 10, 4, 255, 1, 236])




if __name__ == "__main__":

    import sys

    import argparse

    # examples
    # Get one raw sample from the sensor at /dev/ttyACM0 and the serial-number of the sensor DSE0A125
    # python optoforcesensors.py raw --device /dev/ttyACM0 --serial DSE0A125
    # Move to position stored in move file
    # python Mover.py pp --file mov_take_something_with_left_arm.csv - -subset subset_left_arm_and_head.csv --speed 0.1
    # Record move file (trajectory)


    parser = argparse.ArgumentParser()
    parser.add_argument("command",
                        help="One of the commands raw (get raw sensor values), newton (get sensor values in Newton), string ( get the whole message as hex-string-representation ) "
                             +" all (get all data  (time,sample counter, status,x,y,z,checksum)), csv (all data in csv format to store this right away)")
    # fj freeze joints as they are by torquing it. You subset to freeze only a subset of the joints.
    parser.add_argument('--serial', nargs='?', default='DSE0A125',
                        help="serial number of the sensors device")
    parser.add_argument('--device', nargs='?', default='/dev/ttyACM0',
                        help="serial device the sonsor is connected with like /dev/ttyACM0")
    parser.add_argument('--cont', action="store_true", default=False,
                        help="do not stop after one reading")
    #parser.add_argument('--stiffoff', action="store_true", default=False,
    #                    help="sets the stiffness to off after movement")
    args = parser.parse_args()
    # print args

    command = args.command

    optsens = optoforce(args.device,args.serial)

    oneTime=True
    while args.cont or oneTime:

        oneTime=False

        #optsens.get_sensor_values_mh()

        if command == "raw":
            (x, y, z) = optsens.get_sensor_values_raw()
            print "x ,y, z : " + str(x) + "," + str(y) + "," + str(z)

        elif command=="newton":
            (x,y,z) = optsens.get_sensor_values()
            print "x ,y, z (in Newton) : " + str(x) + "," + str(y) + "," + str(z)

        elif command=="string":
            print optsens.get_sensor_string()

        elif command=="all":
            (time,counter, status,x, y, z,checksum) = optsens.get_sensor_all()
            print "time, counter, status,x, y, z,checksum "  + str((time,counter, status,x, y, z,checksum))

        elif command=="csv":
            (time,counter, status,x, y, z,checksum) = optsens.get_sensor_all()
            print str(time)+","+str(counter) +"," + str(status) + "," +str(x) + "," + str(y) + "," + str(z)  + "," + str(checksum)


