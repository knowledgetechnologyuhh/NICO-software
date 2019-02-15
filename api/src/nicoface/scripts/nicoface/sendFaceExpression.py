from time import sleep
import serial
import sys

def sendFaceExpression(expression):
	#ser = serial.Serial('/dev/ttyACM0', 9600)
	ser = serial.Serial('/dev/ttyACM0', 115200) # Establish the connection on a specific port

	try:
		print "Sending " + expression

		ser.write(str(expression)) # Convert the decimal number to ASCII then send it to the Arduino
		print ser.readline() # Read the newest output from the Arduino

		sleep(.2) # Delay for one tenth of a second
		ser.close()
	except:
		print "Could not interface with arduino device. Reason:" + str(sys.exc_info()[0])

if __name__ == "__main__":

	params=sys.argv[1:]
	if (len(params)!=1):
		print "Sends face expressions to NICO Arduino controller."
		print "Use 'python sendFaceExpression.py [happiness,sadness,anger,disgust,surprise,fear,neutral,clean]'"
	else:

		sendFaceExpression(params[0])
