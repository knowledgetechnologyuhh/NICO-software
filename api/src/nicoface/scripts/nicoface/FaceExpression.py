## Author: Erik
##
## ToDos: - Object orientation is bad, please improve


from time import sleep
import serial
import sys
import numpy as np
from PIL import Image, ImageDraw

import numpy as np

import matplotlib.pyplot
import numpy
from PIL import Image



class faceExpression:
    def __init__(self, devicename='/dev/ttyACM0',mode="real"):
        self.mode = mode
        self.comm_mode=2
        self.left = self.create_test_PIL((8,8))
        #self.show_PIL(self.left)
        self.right = self.create_test_PIL((8, 8))
        self.mouth = self.create_test_PIL((16, 8))
        #self.show_PIL(self.mouth)

        if devicename != "sim":
            self.ser = serial.Serial(devicename, 115200)  # Establish the connection on a specific port
            #self.ser = serial.Serial(devicename, 9600)  # Establish the connection on a specific port
            sleep(2)
            #self.send_PIL(self.mouth, "m")
            #raw_input()
        else:
            self.mode = "sim"
            self.ser = None


    def setCommMode(self, mode):
        # Sets the communication mode (0-2)
        # the lower it is, the faster
        # the higher it is, the higher the information from the arduino

        self.comm_mode=mode
        expression="mod"+str(mode)
        print "Sending " + expression

        self.ser.write(str(expression))  # Convert the decimal number to ASCII then send it to the Arduino
        print self.ser.readline()  # Read the newest output from the Arduino

        sleep(.2)  # Delay for one tenth of a second

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

    def send(self,address="all"):

        import time


        if address=="all" or address=="m":
            self.send_PIL(self.mouth, "m")
        #If no comm handshake, waiting time is needed
        if address=="all" and self.comm_mode < 1:
            time.sleep(0.01)
        if address == "all" or address == "l":
            self.send_PIL(self.left, "l")
        if address=="all" and self.comm_mode < 1:
            time.sleep(0.01)
        if address == "all" or address == "r":
            self.send_PIL(self.right, "r")



    def show_PIL(self, Img):
        Img_l = Img.resize((Img.size[0] * 25, Img.size[1] * 25))

        Img_l.show()


    # Send an image to a display. Can be "l" or "r" for left and right elbrow or "m" for mouth
    def send_PIL(self, Img, disp):
        import timeit
        start = timeit.default_timer()
        pix = self.PIL_to_np(Img)
        pix=Img
        lcd_str = self.np_to_str(pix)
        lcd_str = "raw" + disp + lcd_str
        st1=timeit.default_timer()-start
        self.ser.write(str(lcd_str))
        st2 = timeit.default_timer()-start
        if self.comm_mode > 0:
            answ=self.ser.readline()
        st3 = timeit.default_timer() - start
        if self.comm_mode > 1:
            print answ
        st4 = timeit.default_timer()-start
        print "times st1,st2,st3 : " + str(st1) + "," + str(st2) + "," + str(st3) + "," + str(st4)

    def testDisplay(self):
        import random
        # try:
        for t in range(10):
            comm_str = "rawm"
            for m in range(16):
                rn = random.randint(0, 255)
                comm_str += "%0.2X" % rn
            print "Sending " + comm_str

            if self.mode != "sim" :
                self.ser.write(str(comm_str))  # Convert the decimal number to ASCII then send it to the Arduino
                print self.ser.readline()  # Read the newest output from the Arduino

                sleep(.1)  # Delay for one tenth of a second
            else:
                self.show_PIL(self.create_test_PIL((8,8)))
            # except:
            #	print "Could not interface with arduino device. Reason:" + str(sys.exc_info()[0])

    def testObject(self):

        if self.mode != "sim" :
            self.ser.write(str(comm_str))  # Convert the decimal number to ASCII then send it to the Arduino
            print self.ser.readline()  # Read the newest output from the Arduino

            sleep(.1)  # Delay for one tenth of a second
        else:
            self.show_PIL(self.left)
            self.show_PIL(self.right)
            self.show_PIL(self.mouth)
            # except:
            #	print "Could not interface with arduino device. Reason:" + str(sys.exc_info()[0])

    def ricker(self,f, length=0.512, dt=0.001,sca=1,level=0,xoff=0,xstr=1,yoff=0,ystr=1):
        t = np.linspace(-length/2, (length-dt)/2, length/dt)
        y = ((1.-2.*(np.pi**2)*(f**2)*(t**2))*np.exp(-(np.pi**2)*(f**2)*(t**2))*sca)+level
        return t*xstr+xoff, y*ystr+yoff

    def fig2data(self,fig):
        """
        @brief Convert a Matplotlib figure to a 4D numpy array with RGBA channels and return it
        @param fig a matplotlib figure
        @return a numpy 3D array of RGBA values
        http://www.icare.univ-lille1.fr/wiki/index.php/How_to_convert_a_matplotlib_figure_to_a_numpy_array_or_a_PIL_image
        """
        # draw the renderer
        fig.canvas.draw()

        # Get the RGBA buffer from the figure
        w, h = fig.canvas.get_width_height()
        buf = numpy.fromstring(fig.canvas.tostring_argb(), dtype=numpy.uint8)
        buf.shape = (w, h, 4)

        # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
        buf = numpy.roll(buf, 3, axis=2)
        return buf


    def fig2img(self,fig,xsize=16,ysize=8,type="m"):
        """
        @brief Convert a Matplotlib figure to a PIL Image in RGBA format and return it
        @param fig a matplotlib figure
        @return a Python Imaging Library ( PIL ) image
        http://www.icare.univ-lille1.fr/wiki/index.php/How_to_convert_a_matplotlib_figure_to_a_numpy_array_or_a_PIL_image
        """
        # put the figure pixmap into a numpy array

        import PIL

        buf = self.fig2data(fig)
        w, h, d = buf.shape
        image_file=Image.frombytes("RGBA", (w, h), buf.tostring())
        #image_file.convert('1')
        width = image_file.size[0]
        height = image_file.size[1]
        image_file = image_file.crop((0 + width/8,0+height/8,width-width/8,height-height/8))
        #image_file.show()
        #image_file.show()


        ##image_file = image_file.resize((xsize,ysize),PIL.Image.NEAREST)
        #image_file = image_file.resize((16, 8))
        #image_file = image_file.resize((int(xsize*1.8),int(ysize*1.2)))
        image_file = self.binarize_image(image_file,110)
        image_file = image_file.resize((xsize, ysize), PIL.Image.NEAREST)
        #h_width = image_file.size[0]/2
        #h_height = image_file.size[1]/2
        #image_file = image_file.crop((h_width - xsize/2,h_height - ysize/2,h_width+xsize/2,h_height+ysize/2))
        #image_file = image_file.crop((2, 2, xsize , ysize))
        #image_file = image_file.resize((xsize,ysize))

        if type=="m" or type=="r":
            image_file = image_file.rotate(270)

        if type=="l":
            image_file = image_file.rotate(90)

        return image_file

    def fig2png(self,fig,fileName):
        import PIL

        buf = self.fig2data(fig)
        w, h, d = buf.shape
        image_file = Image.frombytes("RGBA", (w, h), buf.tostring())
        # image_file.convert('1')
        width = image_file.size[0]
        height = image_file.size[1]
        image_file = image_file.crop((0 + width / 8, 0 + height / 8, width - width / 8, height - height / 8))
        image_file.save(fileName)


    def binarize_image(self,image, threshold):
        """Binarize an image."""
        image = image.convert('L')  # convert image to monochrome
        image = numpy.array(image)
        image = Image.fromarray(self.binarize_array(image, threshold))
        return image

    def binarize_array(self,numpy_array, threshold=200):
        """Binarize a numpy array."""
        for i in range(len(numpy_array)):
            for j in range(len(numpy_array[0])):
                if numpy_array[i][j] < threshold:
                    numpy_array[i][j] = 255
                else:
                    numpy_array[i][j] = 0
        return numpy_array

    def gen_mouth(self,ml1=(-1.4,0.7,1.0,0),ml2=(None,None,None,None),fileName=None):

        # Generation of the mouth
        # one or two lines can be generated
        # tuple = (stretch_in_y_position , offset_y_position,stretch_in_x_position , offset_x_position,)

        ystr1,yoff1,xstr1,xoff1=ml1
        ystr2, yoff2, xstr2, xoff2 = ml2

        figure = matplotlib.pyplot.figure()
        plot = figure.add_subplot(111)
        figure.patch.set_facecolor('white')

        # plot.ylim((-1.1,1.1))
        # plot.xlim((-1.1,1.1))
        # plt.xlim((min(t),max(t)))

        f = 0.2
        t, y = self.ricker(f, 2, dt=0.0001, ystr=ystr1, yoff=yoff1,xstr=xstr1,xoff=xoff1)
        #t, y = self.ricker(f, 2, dt=0.0001, ystr=0.6, yoff=0.4, xstr=1.0, xoff=0.0)
        lines = plot.plot(t, y, 'k')
        matplotlib.pyplot.setp(lines, linewidth=40, color='k')

        if (ystr2) is not None:
            f = 0.2
            t, y = self.ricker(f, 2, dt=0.0001, ystr=ystr2, yoff=yoff2,xstr=xstr2,xoff=xoff2)
            lines2 = plot.plot(t, y, 'k')
            matplotlib.pyplot.setp(lines2, linewidth=40, color='k')

        matplotlib.pyplot.axis("off")
        matplotlib.pyplot.ylim((-1.1, 1.1))
        matplotlib.pyplot.xlim((-1.1, 1.1))
        #matplotlib.pyplot.show()

        im = self.fig2img(figure,type="m")
        if not fileName is None:
            self.fig2png(figure, fileName)

        return im

    def gen_eyebrowse(self,ml1=(0.1,0.4,1,-0.55), fileName=None,type="l"):

        # Generation of the eyebrowse
        # tuple = (stretch_in_y_position , offset_y_position,stretch_in_x_position , offset_x_position,)

        ystr1,yoff1,xstr1,xoff1=ml1
        if type == "r":
            ystr1 = ystr1 * -1
            #xstr1 = xstr1 * -1

        figure = matplotlib.pyplot.figure()
        plot = figure.add_subplot(111)
        figure.patch.set_facecolor('white')

        # plot.ylim((-1.1,1.1))
        # plot.xlim((-1.1,1.1))
        # plt.xlim((min(t),max(t)))

        f = 0.2
        t, y = self.ricker(f, 4, dt=0.0001, ystr=ystr1, yoff=yoff1,xstr=xstr1,xoff=xoff1)
        #t, y = self.ricker(f, 2, dt=0.0001, ystr=0.6, yoff=0.4, xstr=1.0, xoff=0.0)
        lines = plot.plot(t, y, 'k')
        matplotlib.pyplot.setp(lines, linewidth=20, color='k')

        matplotlib.pyplot.axis("off")
        matplotlib.pyplot.ylim((-1.1, 1.1))
        matplotlib.pyplot.xlim((-1.1, 1.1))
        # matplotlib.pyplot.show()

        im = self.fig2img(figure,xsize=8,ysize=8,type=type)
        im = im.crop((0, 0, 8, 8))

        #self.show_PIL(im)
        #raw_input()

        if not fileName is None:
            self.fig2png(figure, fileName)

        return im

    def test(self):

        imm = self.gen_mouth()

        # pix = np.array(im.getdata()).reshape(8,8)
        #self.show_PIL(imm)

        imb = self.gen_eyebrowse()

        # pix = np.array(im.getdata()).reshape(8,8)
        self.show_PIL(imm)
        self.show_PIL(imb)


        if (self.mode!="sim"):
            self.send_PIL(imm,"m")
            self.send_PIL(imb, "l")
            self.send_PIL(imb, "r")




if __name__ == "__main__":

    params = sys.argv[1:]
    if (len(params) != 1) and (len(params) != 2):
        print "Sends face expressions to NICO Arduino controller."
        print "Use 'python sendFaceExpression.py [happiness,sadness,anger,disgust,surprise,fear,neutral,clean]'"
        #fe = faceExpression()
        #fe.testDisplay()
        #fe = faceExpression("/dev/ttyACM1")
        # fe.testDisplay()
        #fe.test()

    else:
        if params[0] == "test":
            fe = faceExpression("/dev/ttyACM1")
            #fe.testDisplay()
            fe.test()

        else:
            if len(params) == 1:
                fe = faceExpression()
                fe.sendFaceExpression(params[0])
            # sleep(0.5)
            # fe.sendFaceExpression("disgust")
            if len(params) == 2:
                fe = faceExpression(params[1])
                fe.sendFaceExpression(params[0])
