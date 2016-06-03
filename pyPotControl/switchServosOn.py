from lib_robotis import *

dyn = USB2Dynamixel_Device('/dev/ttyUSB0',1000000)

p = Robotis_Servo( dyn, 200 )

p.write_address (24,[1])



