from time import sleep
import serial
import sys
import numpy as np
from PIL import Image, ImageDraw


class faceExpression:
    def __init__(self, devicename='/dev/ttyACM1'):
        self.mode = "real"
        if devicename != "sim":
            self.ser = serial.Serial(devicename, 115200)  # Establish the connection on a specific port
            #self.ser = serial.Serial(devicename, 9600)  # Establish the connection on a specific port
            sleep(2)
        else:
            self.mode = "sim"
            self.ser = None


    def sendFaceExpression(self, expression):
        # try:
        print "Sending " + expression

        self.ser.write(str(expression))  # Convert the decimal number to ASCII then send it to the Arduino
        print self.ser.readline()  # Read the newest output from the Arduino

        sleep(.2)  # Delay for one tenth of a second


    # except:
    #	print "Could not interface with arduino device. Reason:" + str(sys.exc_info()[0])

    def PIL_to_np(self, Img):
        pix = np.array(Img.getdata()).reshape(Img.size[0], Img.size[1])

        return pix


    def np_to_str(self, pix):
        pix_pack = np.packbits(pix)
        lcd_string = ""
        for n in pix_pack:
            lcd_string += "%0.2X" % n
        return (lcd_string)


    def create_test_PIL(self, size_tup):
        Img = Image.new("L", size_tup, 0)
        draw = ImageDraw.Draw(Img)
        draw.line((0, 0, 8, 8), fill=255)
        # draw.line((0, Img.size[1], Img.size[0], 0), fill=255)

        return Img


    def show_PIL(self, Img):
        Img_l = Img.resize((Img.size[0] * 25, Img.size[1] * 25))

        Img_l.show()


    # Send an image to a display. Can be "l" or "r" for left and right elbrow or "m" for mouth
    def send_PIL(self, Img, disp):
        pix = self.PIL_to_np(Img)
        lcd_str = self.np_to_str(pix)
        lcd_str = "raw" + disp + lcd_str
        self.ser.write(str(lcd_str))
        print self.ser.readline()


    def testDisplay(self):
        import random
        # try:
        for t in range(10):
            comm_str = "rawm"
            for m in range(16):
                rn = random.randint(0, 255)
                comm_str += "%0.2X" % rn
            print "Sending " + comm_str

            self.ser.write(str(comm_str))  # Convert the decimal number to ASCII then send it to the Arduino
            print self.ser.readline()  # Read the newest output from the Arduino

            sleep(.1)  # Delay for one tenth of a second

            # except:
            #	print "Could not interface with arduino device. Reason:" + str(sys.exc_info()[0])


if __name__ == "__main__":

    params = sys.argv[1:]
    if (len(params) != 1) and (len(params) != 2):
        print "Sends face expressions to NICO Arduino controller."
        print "Use 'python sendFaceExpression.py [happiness,sadness,anger,disgust,surprise,fear,neutral,clean]'"
    # fe = faceExpression()
    # +fe.testDisplay()
    else:
        if (params[0] == "test"):
            fe = faceExpression("sim")
            fe.testDisplay()
        if (len(params) != 1):
            fe = faceExpression()
            fe.sendFaceExpression(params[0])
        # sleep(0.5)
        # fe.sendFaceExpression("disgust")
        if (len(params) != 2):
            fe = faceExpression(params[1])
            fe.sendFaceExpression(params[0])
