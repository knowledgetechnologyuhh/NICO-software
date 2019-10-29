# Author: Erik, Connor
##
# ToDos: - Object orientation is bad, please improve


import logging
import textwrap
from time import sleep
import numpy as np
import serial
import serial.tools.list_ports
from PIL import Image, ImageDraw


class faceExpression:
    """
    The faceExpression class provides an interface to manipulate NICO's facial
    expressions
    """

    def __init__(self, devicename=None, simulation=False):
        self._logger = logging.getLogger(__name__)

        self.simulation = simulation
        self.comm_mode = 2
        self.left = self.gen_eyebrowse(type="l")
        self.right = self.gen_eyebrowse(type="r")
        self.mouth = self.gen_mouth()

        if not simulation:
            baudrate = 115200
            timeout = 0.016

            if devicename is None:
                self._scan_ports(baudrate, timeout)
            else:
                # Establish the connection on a specific port
                self.ser = serial.Serial(devicename, baudrate, timeout=timeout)
                sleep(2)
        else:
            self.ser = None

    def _scan_ports(self, baudrate, timeout):
        """
        Automatically detects and establishes a connection with the
        FaceExpression Arduino
        """
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer and "Arduino" in p.manufacturer:
                self._logger.info("Connecting to Arduino on port %s", p.device)
                try:
                    self.ser = serial.Serial(p.device, baudrate, timeout=timeout)
                    sleep(1)

                    if self.ser.is_open:
                        self._logger.debug("Trying to send neutral face expression")
                        self.ser.write(b"neutral")
                        response = self.ser.readline()
                        self._logger.debug('Received response: "%s"', repr(response))
                        if response == b"Showing neutral\r\n":
                            self._logger.info(
                                "Successfully connected to FaceExpression "
                                + "device on port %s",
                                p.device,
                            )
                            return
                        self.ser.close()

                except serial.SerialException as e:
                    self._logger.warning(
                        ("Connection to Arduino on port %s failed due to %s"),
                        p.device,
                        e,
                    )

        self._logger.fatal("No FaceExpression Arduino device found")
        self.ser = None

    def _send(self, message, expected_response):
        for _ in range(3):
            self._logger.info("Sending '%s'", message)
            self.ser.write(message.encode("utf-8"))
            response = self.ser.readline()
            if response == expected_response.encode("utf-8"):
                self._logger.info(response[:-2])
                return
            elif response == b"Unknown command. Will not show anything\r\n":
                self._logger.warning("Unknown command %s", message)
                return
            self._logger.debug(
                "Expected response %s but received %s",
                repr(expected_response),
                repr(response),
            )
            self._logger.warning(
                "Failed to send '%s' - resetting serial connection", message
            )
            self.ser.close()
            self.ser.open()
            sleep(1)
        self._logger.critical("Failed to send '%s' after 3 retries", message)
        raise serial.SerialException(
            "Expected response {} but received {}".format(
                repr(expected_response), repr(response)
            )
        )

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

        # Convert the decimal number to ASCII then send it to the Arduino
        # Read the newest output from the Arduino
        self._send(str(expression), "Change Comm mode to  {} \r\n".format(mode))

        sleep(0.2)  # Delay for one tenth of a second

    def sendFaceExpression(self, expression):
        """
        Changes NICO's facial expression to the given preset. The presets
        consist of:
        'happiness','sadness','anger','disgust','surprise','fear','neutral','clear'

        :param expression: name of the desired facial expression (happiness,
        sadness,anger,disgust,surprise,fear,neutral,clear)
        :type expression: str
        """
        # Convert the decimal number to ASCII then send it to the Arduino
        if expression == "clear":
            self._send(expression, "Clearing LCDs\r\n")
        else:
            self._send(expression, "Showing {}\r\n".format(expression))
        sleep(0.2)  # Delay for one tenth of a second

    def sendTrainedFaceExpression(self, expression):
        """
        Changes NICO's facial expression to the given network predicted preset.
        These consist of: 'Angry', 'Happy', 'Neutral', 'Sad', 'Surprise'

        :param expression: name of the desired facial expression ('Angry',
        'Happy', 'Neutral', 'Sad', 'Surprise')
        :type expression: str
        """
        presets = {
            "Angry": {
                "mouth": (
                    (0.99945402, -0.07992669, 0.99940026, 0.01424949),
                    (-0.99829715, -0.11406033, 0.9997558, 0.04432757),
                ),
                "left": (-0.99951923, -0.00889372, 0.99983245, -0.14990053),
                "right": (-0.99873191, 0.08545645, 0.99995756, -0.04182587),
            },
            "Happy": {
                "mouth": (
                    (-0.96794784, -0.01458586, -0.9989453, 0.00975196),
                    (-0.95078206, -0.03179681, 1.0, 0.01479599),
                ),
                "left": (0.99983221, -0.07629592, 1.0, -0.04946393),
                "right": (0.99992925, -0.03617397, 0.99996203, -0.01813084),
            },
            "Neutral": {
                "mouth": (
                    (-0.026799461, -0.50599956, 0.99360126, -0.01208178),
                    (-0.025511968, -0.50718502, 0.99981982, -0.07333233),
                ),
                "left": (0.03521928, 0.0601279, 0.99998277, -0.05035896),
                "right": (0.01149672, 0.0500899, 0.99979389, -0.07785152),
            },
            "Sad": {
                "mouth": (
                    (0.99979699, -0.902700145, 1.0, -0.002130153),
                    (0.99975657, -0.902467377, 1.0, -0.00777484),
                ),
                "left": (0.99999094, -0.03609413, 1.0, -0.05323452),
                "right": (0.99998903, -0.06230563, 0.99999368, -0.01770263),
            },
            "Surprise": {
                "mouth": (
                    (0.99945402, -0.07992669, 0.99940026, 0.01424949),
                    (-0.99829715, -0.11406033, 0.9997558, 0.04432757),
                ),
                "left": (0.99999094, -0.03609413, 1.0, -0.05323452),
                "right": (0.99998903, -0.06230563, 0.99999368, -0.01770263),
            },
        }
        if expression in presets.keys():
            self.mouth = self.gen_mouth(*presets[expression]["mouth"])
            self.left = self.gen_eyebrowse(presets[expression]["left"], type="l")
            self.right = self.gen_eyebrowse(presets[expression]["right"], type="r")

            if self.simulation:
                self.sim_show_face()
            else:
                self.send()

    def sim_show_face(self):
        """
        Displays current face as image
        """
        face = Image.new("L", (24, 16))
        draw = ImageDraw.Draw(face)
        draw.ellipse((9, 1, 13, 5), fill=255)
        draw.ellipse((9, 10, 13, 14), fill=255)

        face.paste(self.left, (0, 8))
        face.paste(self.right, (0, 0))
        face = face.rotate(180)
        face.paste(self.mouth, (0, 0))

        self.show_PIL(face.rotate(90, expand=1))

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
        return lcd_string

    def send(self, address="all"):
        """
        Send current expression ("all") or a specific part of it to display.
        ("m" for mouth, "l" or "r" for left or right eyebrow)

        :param address: part of face to send (all, l, r, m)
        :type address: str
        """
        if address == "all" or address == "m":
            self.send_PIL(self.mouth, "m")
        # If no comm handshake, waiting time is needed
        if address == "all" and self.comm_mode < 1:
            sleep(0.01)
        if address == "all" or address == "l":
            self.send_PIL(self.left, "l")
        if address == "all" and self.comm_mode < 1:
            sleep(0.01)
        if address == "all" or address == "r":
            self.send_PIL(self.right, "r")

    def show_PIL(self, img, scale=25):
        """
        Displays a PIL image

        :param Img: Image to display
        :type Img: PIL.Image
        """
        img_res = img.resize((img.size[0] * scale, img.size[1] * scale))
        img_res.show()

    def send_PIL(self, Img, disp):
        """
        Send an image to a display. Can be "l" or "r" for left and right elbrow
        or "m" for mouth

        :param Img: Image to display
        :type Img: PIL.Image
        :param disp: part of face to send (l, r, m)
        :type disp: str
        """
        pix = Img
        lcd_str = self.np_to_str(pix)
        # generate expected response
        resp = textwrap.wrap(lcd_str, 2)
        resp = "".join(
            ["{} {}".format(i, resp[i].lstrip("0") or "0") for i in range(len(resp))]
        )
        disp_verbose = {"m": "mouth:", "l": "eyebrow l ", "r": "eyebrow r "}[disp]
        resp = "Displaying on {}{} \r\n".format(disp_verbose, resp)
        # generate message
        lcd_str = "raw" + disp + lcd_str
        self._send(str(lcd_str), resp)

    def ricker(self, f, length=0.512, dt=0.001, xoff=0, xstr=1, yoff=0, ystr=1):
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
        y = (1.0 - 2.0 * (np.pi ** 2) * (f ** 2) * (t ** 2)) * np.exp(
            -(np.pi ** 2) * (f ** 2) * (t ** 2)
        )
        return t * xstr + xoff, y * ystr + yoff

    def draw_wavelet(self, length, ystr1, yoff1, xstr1, xoff1, image):
        """
        Generates a ricker wavelet for given parameters and draws them
        into the given PIL image.
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
        :param image: image in which to draw the line
        :type image: PIL image
        :return: modified image
        :rtype: PIL image
        """
        # generate wavelet
        x, y = self.ricker(
            0.2,
            length,
            dt=float(length) / image.size[0],
            ystr=ystr1,
            yoff=yoff1,
            xstr=xstr1,
            xoff=xoff1,
        )
        # map wavelet to image dimensions
        x = range(len(x))
        y = (y + 1) / 2 * image.size[1]
        # draw line
        ImageDraw.Draw(image).line(list(zip(x, y)), fill=255)
        return image

    def gen_mouth(
        self, ml1=(-1.4, 0.7, 1.0, 0), ml2=(None, None, None, None), file_name=None
    ):
        """
        Generation of the mouth. One or two lines can be generated.

        :param ml1: first line of the mouth (stretch_in_y_position,
        offset_y_position, stretch_in_x_position , offset_x_position)
        :type ml1: tuple(float,float,float,float)
        :param ml2: second line of the mouth (stretch_in_y_position,
        offset_y_position, stretch_in_x_position , offset_x_position)
        :type ml2: tuple
        :param file_name: name of the file to save the resulting plot to
        (if desired)
        :type file_name: str
        :return: PIL image of the mouth
        :rtype: PIL.Image
        """
        image = Image.new("L", (16, 8))
        self.draw_wavelet(2, *ml1, image=image)
        if ml2[0] is not None:
            self.draw_wavelet(2, *ml2, image=image)
        image = image.rotate(90, expand=1)

        if file_name:
            image.save(file_name)

        return image

    def gen_eyebrowse(self, ml1=(0.1, 0.4, 1, -0.55), file_name=None, type="l"):
        """
        Generation of the specified eyebrow. The eyebrow is specified by
        setting type to "l" or "r" for left or right eyebrow.

        :param ml1: line of the eyebrow (stretch_in_y_position,
        offset_y_position, stretch_in_x_position , offset_x_position)
        :type ml1: tuple(float,float,float,float)
        :param file_name: name of the file to save the resulting plot to
        (if desired)
        :type file_name: str
        :param type: specifies which eyebrow to generate ("l", "r")
        :type type: str
        """
        ystr1, yoff1, xstr1, xoff1 = ml1
        if type == "r":
            ystr1 *= -1

        image = Image.new("L", (8, 8))
        self.draw_wavelet(2, ystr1, yoff1, xstr1, xoff1, image)

        if type == "r":
            image = image.rotate(90)
        else:
            image = image.rotate(270)

        if file_name:
            image.save(file_name)

        return image
