
from nicotouch import optoforcesensors

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
parser.add_argument('--serial', nargs='?', default=None,
                    help="serial number of the sensors device")
parser.add_argument('--cont', action="store_true", default=False,
                    help="do not stop after one reading")
#parser.add_argument('--stiffoff', action="store_true", default=False,
#                    help="sets the stiffness to off after movement")
args = parser.parse_args()
# print args

command = args.command

optsens = optoforcesensors.optoforce(args.serial)

oneTime=True
while args.cont or oneTime:

    oneTime=False

    #optsens.get_sensor_values_mh()

    if command == "raw":
        (x, y, z) = optsens.get_sensor_values_raw()
        print("x ,y, z : " + str(x) + "," + str(y) + "," + str(z))

    elif command=="newton":
        (x,y,z) = optsens.get_sensor_values()
        print( "x ,y, z (in Newton) : " + str(x) + "," + str(y) + "," + str(z))

    elif command=="string":
        print( optsens.get_sensor_string())

    elif command=="all":
        (time,counter, status,x, y, z,checksum) = optsens.get_sensor_all()
        print("time, counter, status,x, y, z,checksum "  + str((time,counter, status,x, y, z,checksum)))

    elif command=="csv":
        (time,counter, status,x, y, z,checksum) = optsens.get_sensor_all()
        print(str(time)+","+str(counter) +"," + str(status) + "," +str(x) + "," + str(y) + "," + str(z)  + "," + str(checksum))
