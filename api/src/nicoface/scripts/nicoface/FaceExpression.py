# Author: Erik, Connor
##
# ToDos: - Object orientation is bad, please improve


import logging
import sys
from time import sleep

import matplotlib.pyplot
import numpy as np
import serial
import serial.tools.list_ports
from PIL import Image, ImageDraw


class faceExpression:
    """
    The faceExpression class provides an interface to manipulate NICO's facial
    expressions
    """

    def __init__(self, devicename=None, mode="real"):
        self._logger = logging.getLogger(__name__)

        self.mode = mode
        self.comm_mode = 2
	self.baudrate = 115200
	self.timeout = 0.016
        self.left = self.create_test_PIL((8, 8))
        # self.show_PIL(self.left)
        self.right = self.create_test_PIL((8, 8))
        self.mouth = self.create_test_PIL((16, 8))
        # self.show_PIL(self.mouth)

        if devicename != "sim":
            if devicename is None:
                self._scan_ports()
            else:
                # Establish the connection on a specific port
                self.ser = serial.Serial(devicename, self.baudrate, timeout=self.timeout)
                # self.ser = serial.Serial(devicename, 9600)
                sleep(2)
                # self.send_PIL(self.mouth, "m")
                # raw_input()
        else:
            self.mode = "sim"
            self.ser = None

    def _scan_ports(self):
        """
        Automatically detects and establishes a connection with the
        FaceExpression Arduino
        """
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer and "Arduino" in p.manufacturer:
                self._logger.info(
                    "Connecting to Arduino on port {}".format(p.device))
                try:
                    self.ser = serial.Serial(p.device, self.baudrate, timeout=self.timeout)
                    sleep(1)

                    if self.ser.is_open:
                        self._logger.debug(
                            "Trying to send neutral face expression")
                        self.ser.write("neutral")
                        response = self.ser.readline()
                        self._logger.debug(
                            'Received response: "{}"'.format(repr(response)))
                        if response == "Showing neutral\r\n":
                            self._logger.info(
                                "Successfully connected to FaceExpression " +
                                "device on port {}".format(p.device))
                            return
                        self.ser.close()

                except serial.SerialException as e:
                    self._logger.warning(
                        (
                            "Connection to Arduino on port {} failed due to {}"
                        ).format(p.device, e))

        self._logger.fatal("No FaceExpression Arduino device found")
        self.ser = None

    def _send(self, message, expected_response):
	self._logger.info("Sending '{}'".format(message))
	self.ser.write(message)
	response = self.ser.readline()
	self._logger.debug("Received response: '{}'".format(message))
	if response != expected_response:
	    self._logger.warning("Failed to send {} - resetting serial connection".format(message))
	    self.ser.close()
	    self.ser.open()
	    sleep(1)
	    self._logger.info("Resending {}".format(message))
	    self.ser.write(message)
            response = self.ser.readline()
        return response

    def setCommMode(self, mode):
        """
        Sets the communication mode (0-2)
        the lower it is, the faster
        the higher it is, the higher the information from the arduino

        :param mode: communication mode
        :type mode: int
        """
        self.comm_mode = mode
        expression = "mod" + str(mode)
        print("Sending " + expression)

        # Convert the decimal number to ASCII then send it to the Arduino
        print(self._send(str(expression), "Showing  {}\r\n".format(mode)))  # Read the newest output from the Arduino

        sleep(.2)  # Delay for one tenth of a second

    def sendFaceExpression(self, expression):
        """
        Changes NICO's facial expression to the given preset. The presets
        consist of:
        'happiness','sadness','anger','disgust','surprise','fear','neutral','clear'

        :param expression: name of the desired facial expression (happiness,
        sadness,anger,disgust,surprise,fear,neutral,clear)
        :type expression: str
        """
        # try:
        print("Sending " + expression)

        # Convert the decimal number to ASCII then send it to the Arduino
	print(self._send(str(expression), "Showing {}\r\n".format(expression)))
        sleep(.2)  # Delay for one tenth of a second

    # except:
    # print "Could not interface with arduino device. Reason:" +
    # str(sys.exc_info()[0])

    def sendTrainedFaceExpression(self, expression):
        """
        Changes NICO's facial expression to the given network predicted preset.
        These consist of: 'Angry', 'Happy', 'Neutral', 'Sad', 'Surprise'

        :param expression: name of the desired facial expression ('Angry',
        'Happy', 'Neutral', 'Sad', 'Surprise')
        :type expression: str
        """
        presets = {
            'Angry':
            {'mouth': ((0.99945402, -0.07992669,  0.99940026, 0.01424949),
                       (-0.99829715, -0.11406033,  0.9997558, 0.04432757)),
             'left': (-0.99951923, -0.00889372,  0.99983245, -0.14990053),
             'right': (-0.99873191,  0.08545645,  0.99995756, -0.04182587)},
            'Happy':
            {'mouth': ((-0.96794784, -0.01458586, -0.9989453, 0.00975196),
                       (-0.95078206, -0.03179681,  1., 0.01479599)),
             'left': (0.99983221, -0.07629592,  1., -0.04946393),
             'right': (0.99992925, -0.03617397,  0.99996203, -0.01813084)},
            'Neutral':
            {'mouth': ((-0.026799461, -0.50599956,  0.99360126, -0.01208178),
                       (-0.025511968, -0.50718502,  0.99981982, -0.07333233)),
                'left': (0.03521928,  0.0601279,  0.99998277, -0.05035896),
             'right': (0.01149672,  0.0500899,  0.99979389, -0.07785152)},
            'Sad':
            {'mouth': ((0.99979699, -0.902700145, 1.0, -0.002130153),
                       (0.99975657, -0.902467377,  1., -0.00777484)),
             'left': (0.99999094, -0.03609413,  1., -0.05323452),
             'right': (0.99998903, -0.06230563,  0.99999368, -0.01770263)},
            'Surprise':
            {'mouth': ((0.99945402, -0.07992669,  0.99940026, 0.01424949),
                       (-0.99829715, -0.11406033,  0.9997558, 0.04432757)),
             'left': (0.99999094, -0.03609413,  1., -0.05323452),
             'right': (0.99998903, -0.06230563,  0.99999368, -0.01770263)}
        }
        if expression in presets.keys():
            self.mouth = self.gen_mouth(*presets[expression]['mouth'])
            self.left = self.gen_eyebrowse(
                presets[expression]['left'], type='l')
            self.right = self.gen_eyebrowse(
                presets[expression]['right'], type='r')

            if self.mode == "sim":
                self.sim_show_face()
            else:
                self.send()

    def sim_show_face(self):
        face = Image.new('L', (16, 16))

        face.paste(self.mouth, (0, 0))
        face.paste(self.left, (8, 0))
        face.paste(self.right, (8, 8))

        self.show_PIL(face.rotate(90))

    def PIL_to_np(self, Img):
        """
        Converts a PIL image into a numpy array

        :param Img: image to convert
        :type Img: PIL.Image
        :return: Image as numpy array
        :rtype: numpy.array
        """
        pix = np.array(Img.getdata()).reshape(Img.size[0], Img.size[1])

        return pix

    def np_to_str(self, pix):
        """
        Packs a numpy array into a String for serialization

        :param pix: array to pack
        :type pix: numpy.array
        :return: Numpy array packed into a string
        :rtype: str
        """
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

    def send(self, address="all"):
        """
        Send current expression ("all") or a specific part of it to display.
        ("m" for mouth, "l" or "r" for left or right eyebrow)

        :param address: part of face to send (all, l, r, m)
        :type address: str
        """
        import time

        if address == "all" or address == "m":
            self.send_PIL(self.mouth, "m")
        # If no comm handshake, waiting time is needed
        if address == "all" and self.comm_mode < 1:
            time.sleep(0.01)
        if address == "all" or address == "l":
            self.send_PIL(self.left, "l")
        if address == "all" and self.comm_mode < 1:
            time.sleep(0.01)
        if address == "all" or address == "r":
            self.send_PIL(self.right, "r")

    def show_PIL(self, Img):
        """
        Displays a PIL image

        :param Img: Image to display
        :type Img: PIL.Image
        """
        Img_l = Img.resize((Img.size[0] * 25, Img.size[1] * 25))

        Img_l.show()

    def send_PIL(self, Img, disp):
        """
        Send an image to a display. Can be "l" or "r" for left and right elbrow
        or "m" for mouth

        :param Img: Image to display
        :type Img: PIL.Image
        :param disp: part of face to send (l, r, m)
        :type disp: PIL.Image
        """
        import timeit
        start = timeit.default_timer()
        pix = self.PIL_to_np(Img)
        pix = Img
        lcd_str = self.np_to_str(pix)
        lcd_str = "raw" + disp + lcd_str
        st1 = timeit.default_timer() - start
        self.ser.write(str(lcd_str))
        st2 = timeit.default_timer() - start
        if self.comm_mode > 0:
            answ = self.ser.readline()
        st3 = timeit.default_timer() - start
        if self.comm_mode > 1:
            print(answ)
        st4 = timeit.default_timer() - start
        print("times st1,st2,st3 : " + str(st1) + "," +
              str(st2) + "," + str(st3) + "," + str(st4))

    def testDisplay(self):
        import random
        # try:
        for t in range(10):
            comm_str = "rawm"
            for m in range(16):
                rn = random.randint(0, 255)
                comm_str += "%0.2X" % rn
            print("Sending " + comm_str)

            if self.mode != "sim":
                # Convert the decimal number to ASCII then send it to the
                # Arduino
                self.ser.write(str(comm_str))
                # Read the newest output from the Arduino
                print(self.ser.readline())

                sleep(.1)  # Delay for one tenth of a second
            else:
                self.show_PIL(self.create_test_PIL((8, 8)))

    def testObject(self):

        if self.mode != "sim":
            # Convert the decimal number to ASCII then send it to the Arduino
            self.ser.write(str(comm_str))
            # Read the newest output from the Arduino
            print(self.ser.readline())

            sleep(.1)  # Delay for one tenth of a second
        else:
            self.show_PIL(self.left)
            self.show_PIL(self.right)
            self.show_PIL(self.mouth)

    def ricker(self, f, length=0.512, dt=0.001, xoff=0, xstr=1, yoff=0,
               ystr=1):
        """
        Generates a Ricker wavelet (based on
        https://agilescientific.com/blog/2013/12/10/to-plot-a-wavelet.html)

        :param f: peak frequency
        :type f: float
        :param length: timeframe
        :type length: float
        :param dt: samplerate
        :type dt: float
        :param xoff: offset in x position
        :type xoff: float
        :param xstr: stretch in x position
        :type xstr: float
        :param yoff: offset y position
        :type yoff: float
        :param ystr: stretch in y position
        :type ystr: float
        :return: Tuple: time_axis, function_values
        :rtype: tuple
        """
        t = np.linspace(-length / 2, (length - dt) / 2, length / dt)
        y = (1. - 2. * (np.pi**2) * (f**2) * (t**2)) * \
            np.exp(-(np.pi**2) * (f**2) * (t**2))
        return t * xstr + xoff, y * ystr + yoff

    def fig2data(self, fig):
        """
        Convert a Matplotlib figure to a 4D numpy array with RGBA channels and
        return it
        http://www.icare.univ-lille1.fr/wiki/index.php/How_to_convert_a_matplotlib_figure_to_a_numpy_array_or_a_PIL_image

        :param fig: a matplotlib figure
        :return: a numpy 3D array of RGBA values
        :rtype: numpy.array
        """
        # draw the renderer
        fig.canvas.draw()

        # Get the RGBA buffer from the figure
        w, h = fig.canvas.get_width_height()
        buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
        buf.shape = (w, h, 4)

        # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel
        # to have it in RGBA mode
        buf = np.roll(buf, 3, axis=2)
        return buf

    def fig2img(self, fig, xsize=16, ysize=8, type="m"):
        """
        Convert a Matplotlib figure to a PIL Image in RGBA format and return it
        http://www.icare.univ-lille1.fr/wiki/index.php/How_to_convert_a_matplotlib_figure_to_a_numpy_array_or_a_PIL_image

        :param fig: a matplotlib figure
        :return: a Python Imaging Library ( PIL ) image
        :rtype: PIL.Image
        """
        # put the figure pixmap into a numpy array

        import PIL

        buf = self.fig2data(fig)
        w, h, d = buf.shape
        image_file = Image.frombytes("RGBA", (w, h), buf.tostring())
        # image_file.convert('1')
        width = image_file.size[0]
        height = image_file.size[1]
        image_file = image_file.crop(
            (0 + width / 8, 0 + height / 8, width - width / 8,
             height - height / 8))
        # image_file.show()
        # image_file.show()

        # image_file = image_file.resize((xsize,ysize),PIL.Image.NEAREST)
        # image_file = image_file.resize((16, 8))
        # image_file = image_file.resize((int(xsize*1.8),int(ysize*1.2)))
        image_file = self.binarize_image(image_file, 110)
        image_file = image_file.resize((xsize, ysize), PIL.Image.NEAREST)
        # h_width = image_file.size[0]/2
        # h_height = image_file.size[1]/2
        # image_file = image_file.crop((h_width - xsize/2,h_height - ysize/2,
        # h_width+xsize/2,h_height+ysize/2))
        # image_file = image_file.crop((2, 2, xsize , ysize))
        # image_file = image_file.resize((xsize,ysize))

        if type == "m" or type == "r":
            image_file = image_file.rotate(270)

        if type == "l":
            image_file = image_file.rotate(90)

        return image_file

    def fig2png(self, fig, fileName):
        """
        Saves a Matplotlib figure as image file

        :param fig: a matplotlib figure
        :type fig: object
        :param fileName: name of the saved image file
        :type fileName: str
        """
        import PIL

        buf = self.fig2data(fig)
        w, h, d = buf.shape
        image_file = Image.frombytes("RGBA", (w, h), buf.tostring())
        # image_file.convert('1')
        width = image_file.size[0]
        height = image_file.size[1]
        image_file = image_file.crop(
            (0 + width / 8, 0 + height / 8, width - width / 8,
             height - height / 8))
        image_file.save(fileName)

    def binarize_image(self, image, threshold):
        """Binarize an image."""
        image = image.convert('L')  # convert image to monochrome
        image = np.array(image)
        image = Image.fromarray(self.binarize_array(image, threshold))
        return image

    def binarize_array(self, numpy_array, threshold=200):
        """Binarize a numpy array."""
        for i in range(len(numpy_array)):
            for j in range(len(numpy_array[0])):
                if numpy_array[i][j] < threshold:
                    numpy_array[i][j] = 255
                else:
                    numpy_array[i][j] = 0
        return numpy_array

    def gen_mouth(self, ml1=(-1.4, 0.7, 1.0, 0), ml2=(None, None, None, None),
                  fileName=None):
        """
        Generation of the mouth. One or two lines can be generated.

        :param ml1: first line of the mouth (stretch_in_y_position,
        offset_y_position, stretch_in_x_position , offset_x_position)
        :type ml1: tuple(float,float,float,float)
        :param ml2: second line of the mouth (stretch_in_y_position,
        offset_y_position, stretch_in_x_position , offset_x_position)
        :type ml2: tuple
        :param fileName: name of the file to save the resulting plot to
        (if desired)
        :type fileName: str
        :return: PIL image of the mouth
        :rtype: PIL.Image
        """

        ystr1, yoff1, xstr1, xoff1 = ml1
        ystr2, yoff2, xstr2, xoff2 = ml2

        figure = matplotlib.pyplot.figure()
        plot = figure.add_subplot(111)
        figure.patch.set_facecolor('white')

        # plot.ylim((-1.1,1.1))
        # plot.xlim((-1.1,1.1))
        # plt.xlim((min(t),max(t)))

        f = 0.2
        t, y = self.ricker(f, 2, dt=0.0001, ystr=ystr1,
                           yoff=yoff1, xstr=xstr1, xoff=xoff1)
        # t, y = self.ricker(f, 2, dt=0.0001, ystr=0.6, yoff=0.4, xstr=1.0,
        # xoff=0.0)
        lines = plot.plot(t, y, 'k')
        matplotlib.pyplot.setp(lines, linewidth=40, color='k')

        if (ystr2) is not None:
            f = 0.2
            t, y = self.ricker(f, 2, dt=0.0001, ystr=ystr2,
                               yoff=yoff2, xstr=xstr2, xoff=xoff2)
            lines2 = plot.plot(t, y, 'k')
            matplotlib.pyplot.setp(lines2, linewidth=40, color='k')

        matplotlib.pyplot.axis("off")
        matplotlib.pyplot.ylim((-1.1, 1.1))
        matplotlib.pyplot.xlim((-1.1, 1.1))
        # matplotlib.pyplot.show()

        im = self.fig2img(figure, type="m")
        if fileName:
            self.fig2png(figure, fileName)

        return im

    def gen_eyebrowse(self, ml1=(0.1, 0.4, 1, -0.55), fileName=None, type="l"):
        """
        Generation of the specified eyebrow. The eyebrow is specified by
        setting type to "l" or "r" for left or right eyebrow.

        :param ml1: line of the eyebrow (stretch_in_y_position,
        offset_y_position, stretch_in_x_position , offset_x_position)
        :type ml1: tuple(float,float,float,float)
        :param fileName: name of the file to save the resulting plot to
        (if desired)
        :type fileName: str
        :param type: specifies which eyebrow to generate ("l", "r")
        :type type: str
        """

        ystr1, yoff1, xstr1, xoff1 = ml1
        if type == "r":
            ystr1 = ystr1 * -1
            # xstr1 = xstr1 * -1

        figure = matplotlib.pyplot.figure()
        plot = figure.add_subplot(111)
        figure.patch.set_facecolor('white')

        # plot.ylim((-1.1,1.1))
        # plot.xlim((-1.1,1.1))
        # plt.xlim((min(t),max(t)))

        f = 0.2
        t, y = self.ricker(f, 4, dt=0.0001, ystr=ystr1,
                           yoff=yoff1, xstr=xstr1, xoff=xoff1)
        # t, y = self.ricker(f, 2, dt=0.0001, ystr=0.6, yoff=0.4,
        # xstr=1.0, xoff=0.0)
        lines = plot.plot(t, y, 'k')
        matplotlib.pyplot.setp(lines, linewidth=20, color='k')

        matplotlib.pyplot.axis("off")
        matplotlib.pyplot.ylim((-1.1, 1.1))
        matplotlib.pyplot.xlim((-1.1, 1.1))
        # matplotlib.pyplot.show()

        im = self.fig2img(figure, xsize=8, ysize=8, type=type)
        im = im.crop((0, 0, 8, 8))

        # self.show_PIL(im)
        # raw_input()

        if fileName:
            self.fig2png(figure, fileName)

        return im

    def test(self):

        imm = self.gen_mouth()

        # pix = np.array(im.getdata()).reshape(8,8)
        # self.show_PIL(imm)

        imb = self.gen_eyebrowse()

        # pix = np.array(im.getdata()).reshape(8,8)
        self.show_PIL(imm)
        self.show_PIL(imb)

        if (self.mode != "sim"):
            self.send_PIL(imm, "m")
            self.send_PIL(imb, "l")
            self.send_PIL(imb, "r")


if __name__ == "__main__":

    params = sys.argv[1:]
    if (len(params) != 1) and (len(params) != 2):
        print("Sends face expressions to NICO Arduino controller.")
        print(
            "Use 'python sendFaceExpression.py [happiness,sadness,anger," +
            "disgust,surprise,fear,neutral,clean]'")
        # fe = faceExpression()
        # fe.testDisplay()
        # fe = faceExpression("/dev/ttyACM1")
        # fe.testDisplay()
        # fe.test()

    else:
        if params[0] == "test":
            fe = faceExpression("/dev/ttyACM1")
            # fe.testDisplay()
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
