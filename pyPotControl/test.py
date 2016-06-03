import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()

if not ports:
    raise IOError('no port found!')

print('ports found', ports)

print('connecting on the first available port:', ports[0])

dxl_io = pypot.dynamixel.DxlIO(ports[0])

ids = dxl_io.scan(range(27))

print(dxl_io.get_present_position(ids))


for id in ids:
	print "ID: " + str(id) + " angle: " + str(dxl_io.get_present_position([id]))


#dxl_io.set_goal_position({21: 2.00})
#dxl_io.set_goal_position({22: -12.00})

#print(dxl_io.get_present_position(ids))
