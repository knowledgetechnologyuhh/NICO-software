# Authors: Erik, Connor


import logging
import textwrap
from time import sleep
import numpy as np
from serial import SerialException
from SerialConnectionManager import SerialDevice
from PIL import Image, ImageDraw
import cv2


class faceExpression:
    """
    The FaceExpression class provides an interface to manipulate NICO's facial
    expressions
    """

    trained_presets = {
        "anger": {
            "mouth": (
                (0.99945402, -0.07992669, 0.99940026, 0.01424949),
                (-0.99829715, -0.11406033, 0.9997558, 0.04432757),
            ),
            "left": (-0.99951923, -0.00889372, 0.99983245, -0.14990053),
            "right": (-0.99873191, 0.08545645, 0.99995756, -0.04182587),
        },
        "happiness": {
            "mouth": (
                (-0.96794784, -0.01458586, -0.9989453, 0.00975196),
                (-0.95078206, -0.03179681, 1.0, 0.01479599),
            ),
            "left": (0.99983221, -0.07629592, 1.0, -0.04946393),
            "right": (0.99992925, -0.03617397, 0.99996203, -0.01813084),
        },
        "neutral": {
            "mouth": (
                (-0.026799461, -0.50599956, 0.99360126, -0.01208178),
                (-0.025511968, -0.50718502, 0.99981982, -0.07333233),
            ),
            "left": (0.03521928, 0.0601279, 0.99998277, -0.05035896),
            "right": (0.01149672, 0.0500899, 0.99979389, -0.07785152),
        },
        "sadness": {
            "mouth": (
                (0.99979699, -0.902700145, 1.0, -0.002130153),
                (0.99975657, -0.902467377, 1.0, -0.00777484),
            ),
            "left": (0.99999094, -0.03609413, 1.0, -0.05323452),
            "right": (0.99998903, -0.06230563, 0.99999368, -0.01770263),
        },
        "surprise": {
            "mouth": (
                (0.99945402, -0.07992669, 0.99940026, 0.01424949),
                (-0.99829715, -0.11406033, 0.9997558, 0.04432757),
            ),
            "left": (0.99999094, -0.03609413, 1.0, -0.05323452),
            "right": (0.99998903, -0.06230563, 0.99999368, -0.01770263),
        },
    }

    polynomial_presets = {
        "happiness": {
            "mouth": [[7, 0, -0.11, 0, 0], [7, 0, -0.11, 0, 0], 7.45, 2, 2],
            "left": [[4.75, -0.25, 0, 0, 0], 0, 0, 0],
            "right": [[3, 0.25, 0, 0, 0], 0, 0, 0],
        },
        "sadness": {
            "mouth": [[5, 0, 0.075, 0, 0], [5, 0, 0.075, 0, 0], 7.45, 2, 2],
            "left": [[5, -0.4, 0, 0, 0], 0, 0, 0],
            "right": [[2.2, 0.4, 0, 0, 0], 0, 0, 0],
        },
        "anger": {
            "mouth": [[2, 0, 0.005, 0, 0.002], [8, 0, -0.005, 0, -0.002], 7.45, 2, 2],
            "left": [[2.5, 0.7, 0, 0, 0], 0, 0, 0],
            "right": [[7.4, -0.7, 0, 0, 0], 0, 0, 0],
        },
        "disgust": {
            "mouth": [[4, 0, -0.39, 0, 0.0385], [7, 0, 0.39, 0, -0.0385], 7.45, 4, 4],
            "left": [[8.75, -0.25, 0, 0, 0], 0, 0, 0],
            "right": [[7, 0.25, 0, 0, 0], 0, 0, 0],
        },
        "surprise": {
            "mouth": [[2, 0, 0.005, 0, 0.0045], [8, 0, -0.005, 0, -0.0045], 7.45, 3, 3],
            "left": [[5, -1, 0.125, 0, 0], 0, 0, 0],
            "right": [[4, -0.5, 0.095, 0, 0], 0, 0, 0],
        },
        "fear": {
            "mouth": [
                [3, 0, -0.005, 0.0, 0.00075],
                [7, 0, 0.005, 0.0, -0.00075],
                7.45,
                1,
                1,
            ],
            "left": [[3, -0.25, 0, 0, 0], 0, 0, 0],
            "right": [[1.25, 0.25, 0, 0, 0], 0, 0, 0],
        },
        "neutral": {
            "mouth": [[5, 0, 0, 0, 0], [5, 0, 0, 0, 0], 7.45, 2, 2],
            "left": [[4, 0, 0, 0, 0], 0, 0, 0],
            "right": [[4, 0, 0, 0, 0], 0, 0, 0],
        },
    }

    def __init__(self, devicename=None, simulation=False):
        """
        faceExpression provides an interface to generate and display face
        expressions on the NICO head

        :param devicename: serial device to connect to (autodetected if None)
        :type devicename: str
        :param simulation: If enabled, faces will be displayed as images instead
                           of sending them to the serial device
        :type simulation: bool
        """
        self._logger = logging.getLogger(__name__)
        self._presets = (
            "happiness",
            "sadness",
            "anger",
            "disgust",
            "surprise",
            "fear",
            "neutral",
        )
        self.simulation = simulation
        self.comm_mode = 2
        self._parameters = {}
        self._basis_functions = ["", "", ""]
        self.generate_polynomial_eyebrow(*self.polynomial_presets["neutral"]["left"])
        self.generate_polynomial_eyebrow(
            *self.polynomial_presets["neutral"]["right"], left=False
        )
        self.generate_polynomial_mouth(*self.polynomial_presets["neutral"]["mouth"])

        if not simulation:
            baudrate = 115200
            timeout = 0.016

            if devicename is None:
                self._scan_ports(baudrate, timeout)
            else:
                # Establish the connection on a specific port
                self.ser = SerialDevice(devicename, baudrate, timeout=timeout)
                self.sendFaceExpression("neutral")
            self.is_morphable = False
        else:
            self.ser = None
            self.is_morphable = True
            self.send()

    def _scan_ports(self, baudrate, timeout):
        """
        Automatically detects and establishes a connection with the
        FaceExpression Arduino
        """
        devices = SerialDevice.get_devices_by_manufacturer("duino")
        for device in devices:
            self._logger.info("Connecting to Arduino on port %s", device)
            try:
                self.ser = SerialDevice(device, baudrate, timeout=timeout)

                self._logger.debug("Trying to send neutral face expression")
                response = self.ser.send("neutral")
                if response == b"Showing neutral\r\n":
                    self._logger.info(
                        "Successfully connected to FaceExpression "
                        + "device on port %s",
                        device,
                    )
                    return
                self.ser.close()

            except SerialException as e:
                self._logger.warning(
                    ("Connection to Arduino on port %s failed due to %s"), device, e
                )

        self._logger.fatal("No FaceExpression Arduino device found")
        self.ser = None
        exit(1)

    def _send(self, message, expected_response):
        """
        Sends message to serial device and verifies if expected_response was
        returned

        :param message: message to send
        :type message: str
        :param expected_response: response the serial device is expected to return
        :type expected_response: str
        """
        for _ in range(3):
            response = self.ser.send(message)
            if response == expected_response.encode("utf-8"):
                self._logger.debug("Received expected response %s", repr(response))
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
            self.ser.reset()
        self._logger.critical("Failed to send '%s' after 3 retries", message)
        raise SerialException(
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
        self._logger.info("Setting communication mode to %i", mode)
        self.comm_mode = mode
        expression = "mod" + str(mode)

        # Convert the decimal number to ASCII then send it to the Arduino
        # Read the newest output from the Arduino
        self._send(str(expression), "Change Comm mode to  {} \r\n".format(mode))

    def sendFaceExpression(self, expression):
        """
        Changes NICO's facial expression to the given preset. The presets
        consist of:
        'happiness','sadness','anger','disgust','surprise','fear','neutral','clear'

        :param expression: name of the desired facial expression (happiness,
                           sadness,anger,disgust,surprise,fear,neutral,clear)
        :type expression: str
        """
        if self.simulation:
            self._logger.warning(
                "'sendFaceExpression' does not work in simulated mode - "
                "use 'send_morphable_face_expression' or "
                "'sendTrainedFaceExpression' instead"
            )
            return
        self._logger.info("Showing expression: '%s'", expression)
        if expression == "clear":
            self._send(expression, "Clearing LCDs\r\n")
        elif expression in self._presets:
            self._send(expression, "Showing {}\r\n".format(expression))
        else:
            self._logger.warning("Unknown expression '%s'", expression)
        self.is_morphable = False

    def sendTrainedFaceExpression(self, expression):
        """
        Changes NICO's facial expression to the given network predicted preset.
        These consist of: 'anger', 'happiness', 'neutral', 'sadness', 'surprise'

        :param expression: name of the desired facial expression ('anger',
                           'happiness', 'neutral', 'sadness', 'surprise')
        :type expression: str
        """

        if expression in self.trained_presets.keys():
            self._logger.info("Showing trained expression: '%s'", expression)
            self.send_wavelet_face(
                *self.trained_presets[expression]["mouth"],
                self.trained_presets[expression]["left"],
                self.trained_presets[expression]["right"],
            )
        else:
            self._logger.warning("Unknown expression '%s'", expression)

    def sim_show_face(self):
        """
        Displays current face as image
        """
        self._logger.debug("Creating full face image")
        face = Image.new("L", (24, 18))
        draw = ImageDraw.Draw(face)
        draw.ellipse((9, 2, 13, 6), fill=255)
        draw.ellipse((9, 11, 13, 15), fill=255)

        face.paste(self.left, (0, 10))
        face.paste(self.right, (0, 0))
        face = face.rotate(180)
        face.paste(self.mouth, (0, 1))

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
        if self.simulation:
            self.sim_show_face()
        else:
            if address not in ("all", "l", "r", "m"):
                self._logger.warning("Unknown display %s", address)
                return
            self._logger.info("Showing custom image on '%s'", address)
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
        self.is_morphable = True

    def show_PIL(self, img, scale=25):
        """
        Displays a PIL image

        :param Img: Image to display
        :type Img: PIL.Image
        """
        self._logger.debug("Displaying face image")
        img_res = img.resize((img.size[0] * scale, img.size[1] * scale))
        img_cv = np.array(img_res, dtype=np.uint8)
        cv2.imshow("Simulated Face Expression", img_cv)
        cv2.waitKey(40)

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
        t = np.linspace(
            -length / 2 * xstr + xoff, (length - dt) / 2 * xstr + xoff, int(length / dt)
        )
        y = (1.0 - 2.0 * (np.pi ** 2) * (f ** 2) * (t ** 2)) * np.exp(
            -(np.pi ** 2) * (f ** 2) * (t ** 2)
        )
        return t, y * ystr + yoff

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

        self._basis_functions[0] = "wavelet"
        self._parameters["mouth"] = ml1, ml2
        self.mouth = image

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
            yoff1 *= -1

        image = Image.new("L", (8, 8))
        self.draw_wavelet(2, ystr1, yoff1, xstr1, xoff1, image)

        if type == "r":
            image = image.rotate(90)
            self._basis_functions[2] = "wavelet"
            self._parameters["right"] = ml1
            self.right = image
        else:
            image = image.rotate(270)
            self._basis_functions[1] = "wavelet"
            self._parameters["left"] = ml1
            self.left = image

        if file_name:
            image.save(file_name)

    def send_bitmap_face(self, brow_left=None, brow_right=None, mouth=None):
        """
        Displays given bitmaps on the face

        :param brow_left: 8x8 bitmap to display as the left brow
        :type brow_left: np.array (shape=(8,8))
        :param brow_right: 8x8 bitmap to display as the right brow
        :type brow_right: np.array (shape=(8,8))
        :param mouth: 16x8 bitmap to display as the mouth
        :type brow_left: np.array (shape=(8,16))
        """
        if brow_left is not None:
            self._basis_functions[1] = "bitmap"
            if type(brow_left) != np.ndarray or brow_left.shape != (8, 8):
                self._logger.error(
                    "'brow_left' needs to be an np.array with shape (8,8)"
                )
                raise ValueError("'brow_left' needs to be an np.array with shape (8,8)")
            else:
                self.left = Image.fromarray(brow_left * 255).rotate(90)
        if brow_right is not None:
            self._basis_functions[2] = "bitmap"
            if type(brow_right) != np.ndarray or brow_right.shape != (8, 8):
                self._logger.error(
                    "'brow_right' needs to be an np.array with shape (8,8)"
                )
                raise ValueError(
                    "'brow_right' needs to be an np.array with shape (8,8)"
                )
            else:
                self.right = Image.fromarray(brow_right * 255).rotate(90)
        if mouth is not None:
            self._basis_functions[0] = "bitmap"
            if type(mouth) != np.ndarray or mouth.shape != (8, 16):
                self._logger.error("'mouth' needs to be an np.array with shape (8,16)")
                raise ValueError("'mouth' needs to be an np.array with shape (8,16)")
            else:
                self.mouth = Image.fromarray(mouth * 255).rotate(270, expand=1)

        self.send()

    def polynomial(self, x, degs):
        """
        Calculates y values for given x values using an n-degree polynomial:
        y = sum(degs[i] * x ** i)

        :param x: x values to calculate y for
        :type x: list
        :param degs: factor for each degree of the polynomial
        :type degs: list
        """
        return sum([degs[i] * (x ** i) for i in range(len(degs))])

    def generate_polynomial_mouth(
        self, degs1, degs2, x_shift=7.45, crop_left=0, crop_right=0
    ):
        """
        Generates mouth curves using two n-degree polynomial y = sum(degs[i] * x ** i).
        The origin point can be shifted with x_shift, the crop_left and crop_right
        parameters allow creating black borders at either side of the image to reduce
        size of the mouth.

        :param degs1: factor for each degree of the first polynomial mouth line
        :type degs1: list
        :param degs2: factor for each degree of the second polynomial mouth line
        :type degs2: list
        :param x_shift: reduces x values and therefore shifts curve to the right (x = -x_shift + i)
        :type x_shift: float
        :param crop_left: black border in pixels on the left of the resulting image
        :type crop_left: int
        :param crop_right: black border in pixels on the right of the resulting image
        :type crop_right: int
        """
        x = np.arange(16)
        y1 = self.polynomial(x - x_shift, degs1)
        y2 = self.polynomial(x - x_shift, degs2)

        img = Image.new("L", (16, 8))
        ImageDraw.Draw(img).line(list(zip(x, y1)), fill=255)
        ImageDraw.Draw(img).line(list(zip(x, y2)), fill=255)
        generated = img.rotate(270, expand=1)

        generated.paste(Image.new("L", (8, crop_left)), (0, 0))
        generated.paste(Image.new("L", (8, crop_right)), (0, 16 - crop_right))

        self.mouth = generated
        self._parameters["mouth"] = degs1, degs2, x_shift, crop_left, crop_right
        self._basis_functions[0] = "polynomial"

    def generate_polynomial_eyebrow(
        self, degs, x_shift=0, crop_left=0, crop_right=0, left=True
    ):
        """
        Generates eyebrow curve using an n-degree polynomial y = sum(degs[i] * x ** i).
        The origin point can be shifted with x_shift, the crop_left and crop_right
        parameters allow creating black borders at either side of the image to reduce
        size of the eyebrow.

        :param degs: factor for each degree of the polynomial
        :type degs: list
        :param x_shift: reduces x values and therefore shifts curve to the right (x = -x_shift + i)
        :type x_shift: float
        :param crop_left: black border in pixels on the left of the resulting image
        :type crop_left: int
        :param crop_right: black border in pixels on the right of the resulting image
        :type crop_right: int
        :param left: whether left or right eyebrow is generated
        :type left: bool
        """
        x = np.arange(8)
        y = self.polynomial(x - x_shift, degs)

        img = Image.new("L", (8, 8))
        ImageDraw.Draw(img).line(list(zip(x, y)), fill=255)
        generated = img.rotate(90)

        generated.paste(Image.new("L", (8, crop_left)), (0, 0))
        generated.paste(Image.new("L", (8, crop_right)), (0, 8 - crop_right))

        if left:
            self.left = generated
            self._parameters["left"] = degs, x_shift, crop_left, crop_right
            self._basis_functions[1] = "polynomial"
        else:
            self.right = generated
            self._parameters["right"] = degs, x_shift, crop_left, crop_right
            self._basis_functions[2] = "polynomial"

    def send_morphable_face_expression(self, expression):
        """
        Changes NICO's facial expression to the given preset. The presets
        consist of:
        'happiness','sadness','anger','disgust','surprise','fear','neutral'

        These presets are used for morph_face_expression. They are slightly altered
        versions of the regular face expressions.

        :param expression: name of the desired facial expression (happiness,
                           sadness,anger,disgust,surprise,fear,neutral)
        :type expression: str
        """
        self._logger.debug("Sending polynomial preset '%s'" % expression)
        self.send_polynomial_face(
            *self.polynomial_presets[expression]["mouth"],
            *self.polynomial_presets[expression]["left"],
            *self.polynomial_presets[expression]["right"],
        )

    def send_wavelet_face(self, m1, m2, left, right):
        """
        Generates and displays face expression with given wavelet paramters.

        :param m1: wavelet parameters for first mouth line (stretch_in_y_position,
                          offset_y_position, stretch_in_x_position , offset_x_position)
        :type m1: tuple(float, float, float, float)
        :param m2: wavelet parameters for second mouth line
        :type m2: tuple(float, float, float, float)
        :param left: wavelet parameters for left eyebrow
        :type left: tuple(float, float, float, float)
        :param right: wavelet paramters for right eyebrow
        :type right: tuple(float, float, float, float)
        """
        self._logger.debug("Displaying wavelet face")
        # show mouth
        self.gen_mouth(m1, m2)
        # show left
        self.gen_eyebrowse(left, type="l")
        # show right
        self.gen_eyebrowse(right, type="r")
        self.send()

    def send_polynomial_face(
        self,
        m1,
        m2,
        m_x_shift,
        m_crop_left,
        m_crop_right,
        left,
        l_x_shift,
        l_crop_left,
        l_crop_right,
        right,
        r_x_shift,
        r_crop_left,
        r_crop_right,
    ):
        """
        Generates and displays face with the given polynomials.

        :param m1: factor for each degree of the first polynomial mouth line
        :type m1: list
        :param m2: factor for each degree of the second polynomial mouth line
        :type m2: list
        :param m_x_shift: reduces x values and therefore shifts curve to the right (x = -x_shift + i)
        :type m_x_shift: float
        :param m_crop_left: black border in pixels on the left of the resulting mouth image
        :type m_crop_left: int
        :param m_crop_right: black border in pixels on the right of the resulting mouth image
        :type m_crop_right: int
        :param left: factor for each degree of the left eyebrow polynomial
        :type left: list
        :param l_x_shift: reduces x values and therefore shifts curve to the right (x = -x_shift + i)
        :type l_x_shift: float
        :param l_crop_left: black border in pixels on the left of the resulting eyebrow image
        :type l_crop_left: int
        :param l_crop_right: black border in pixels on the right of the resulting eyebrow image
        :type l_crop_right: int
        :param right: factor for each degree of the right eyebrow polynomial
        :type right: list
        :param r_x_shift: reduces x values and therefore shifts curve to the right (x = -x_shift + i)
        :type r_x_shift: float
        :param r_crop_left: black border in pixels on the left of the resulting eyebrow image
        :type r_crop_left: int
        :param r_crop_right: black border in pixels on the right of the resulting eyebrow image
        :type r_crop_right: int
        """
        self._logger.debug("Displaying polynomial face")
        # show mouth
        self.generate_polynomial_mouth(m1, m2, m_x_shift, m_crop_left, m_crop_right)
        # show left
        self.generate_polynomial_eyebrow(left, l_x_shift, l_crop_left, l_crop_right)
        # show right
        self.generate_polynomial_eyebrow(
            right, r_x_shift, r_crop_left, r_crop_right, False
        )
        self.send()

    def morph_face_expression(self, target_preset, steps=3, delay=0.0):
        """
        Morphs displayed face to given preset. This requires the displayed face
        expression to be fully polynomial or fully wavelet based.

        Known presets: 'happiness', 'sadness', 'anger', 'disgust', 'surprise',
                       'fear', 'neutral'

        :param target_preset: name of the target expression ('happiness', 'sadness',
                              'anger', 'disgust', 'surprise', 'fear', 'neutral')
        :type target_preset: str
        :param steps: number of transition steps
        :type steps: int
        :param delay: delay between transition steps
        :type delay: float
        """
        if self._basis_functions[0] == "polynomial":
            self.morph_polynomial_face(
                *self.polynomial_presets[target_preset]["mouth"],
                *self.polynomial_presets[target_preset]["left"],
                *self.polynomial_presets[target_preset]["right"],
                steps,
                delay,
            )
        else:
            self.morph_wavelet_face(
                *self.trained_presets[target_preset]["mouth"],
                self.trained_presets[target_preset]["left"],
                self.trained_presets[target_preset]["right"],
                steps,
                delay,
            )

    def morph_polynomial_face(
        self,
        m1_target,
        m2_target,
        m_x_shift_target,
        m_crop_left_target,
        m_crop_right_target,
        left_target,
        l_x_shift_target,
        l_crop_left_target,
        l_crop_right_target,
        right_target,
        r_x_shift_target,
        r_crop_left_target,
        r_crop_right_target,
        steps=3,
        delay=0.0,
    ):
        """
        Morphs displayed face into the given target face in the given number of
        transition steps. This requires all face images to be polynomial based.

        :param m1_target: factor for each degree of the first polynomial mouth line
        :type m1_target: list
        :param m2_target: factor for each degree of the second polynomial mouth line
        :type m2_target: list
        :param m_x_shift_target: reduces x values and therefore shifts curve to the right (x = -x_shift + i)
        :type m_x_shift_target: float
        :param m_crop_left_target: black border in pixels on the left of the resulting mouth image
        :type m_crop_left_target: int
        :param m_crop_right_target: black border in pixels on the right of the resulting mouth image
        :type m_crop_right_target: int
        :param left_target: factor for each degree of the left eyebrow polynomial
        :type left_target: list
        :param l_x_shift_target: reduces x values and therefore shifts curve to the right (x = -x_shift + i)
        :type l_x_shift_target: float
        :param l_crop_left_target: black border in pixels on the left of the resulting eyebrow image
        :type l_crop_left_target: int
        :param l_crop_right_target: black border in pixels on the right of the resulting eyebrow image
        :type l_crop_right_target: int
        :param right_target: factor for each degree of the right eyebrow polynomial
        :type right_target: list
        :param r_x_shift_target: reduces x values and therefore shifts curve to the right (x = -x_shift + i)
        :type r_x_shift_target: float
        :param r_crop_left_target: black border in pixels on the left of the resulting eyebrow image
        :type r_crop_left_target: int
        :param r_crop_right_target: black border in pixels on the right of the resulting eyebrow image
        :type r_crop_right_target: int
        :param steps: number of transition steps
        :type steps: int
        :param delay: delay between transition steps
        :type delay: float
        """
        # check if current face is polynomial based
        if not self.is_morphable:
            self._logger.error("Current face expression does not support morphing")
            raise ValueError("Current face expression does not support morphing")
        if "bitmap" in self._basis_functions:
            self._logger.error("Face contains images generated directly from a bitmap.")
            raise ValueError("Face contains images generated directly from a bitmap.")
        if "wavelet" in self._basis_functions:
            self._logger.error(
                "Missmatched basis functions, face contains both 'polynomial' "
                "and 'wavelet' based images"
            )
            raise ValueError(
                "Missmatched basis functions, face contains both 'polynomial' "
                "and 'wavelet' based images"
            )

        mouth_start = self._parameters["mouth"]
        left_start = self._parameters["left"]
        right_start = self._parameters["right"]

        # mouth
        self._logger.debug("Calculating mouth transitions")
        m1 = self._calculate_transition(mouth_start[0], m1_target)
        m2 = self._calculate_transition(mouth_start[1], m2_target)
        m_x_shift, m_crop_left, m_crop_right = zip(
            *self._calculate_transition(
                mouth_start[2:],
                (m_x_shift_target, m_crop_left_target, m_crop_right_target),
            )
        )
        # left
        self._logger.debug("Calculating left eyebrow transitions")
        left = self._calculate_transition(left_start[0], left_target)
        l_x_shift, l_crop_left, l_crop_right = zip(
            *self._calculate_transition(
                left_start[1:],
                (l_x_shift_target, l_crop_left_target, l_crop_right_target),
            )
        )
        # right
        self._logger.debug("Calculating right eyebrow transitions")
        right = self._calculate_transition(right_start[0], right_target)
        r_x_shift, r_crop_left, r_crop_right = zip(
            *self._calculate_transition(
                right_start[1:],
                (r_x_shift_target, r_crop_left_target, r_crop_right_target),
            )
        )

        # show intermediate faces
        self._logger.debug("Displaying transition faces")
        for step in range(steps):
            self.send_polynomial_face(
                m1[step],
                m2[step],
                m_x_shift[step],
                int(m_crop_left[step]),
                int(m_crop_right[step]),
                left[step],
                l_x_shift[step],
                int(l_crop_left[step]),
                int(l_crop_right[step]),
                right[step],
                r_x_shift[step],
                int(r_crop_left[step]),
                int(r_crop_right[step]),
            )
            sleep(delay)
        # show final face
        self._logger.debug("Displaying target face")
        self.send_polynomial_face(
            m1_target,
            m2_target,
            m_x_shift_target,
            m_crop_left_target,
            m_crop_right_target,
            left_target,
            l_x_shift_target,
            l_crop_left_target,
            l_crop_right_target,
            right_target,
            r_x_shift_target,
            r_crop_left_target,
            r_crop_right_target,
        )

    def morph_wavelet_face(
        self, m1_target, m2_target, left_target, right_target, steps=3, delay=0.0
    ):
        """
        Morphs displayed face into the given target face in the given number of
        transition steps. This requires all face images to be wavelet based.

        :param m1_target: wavelet parameters for first mouth line (stretch_in_y_position,
                          offset_y_position, stretch_in_x_position , offset_x_position)
        :type m1_target: tuple(float, float, float, float)
        :param m2_target: wavelet parameters for second mouth line
        :type m2_target: tuple(float, float, float, float)
        :param left_target: wavelet parameters for left eyebrow
        :type left_target: tuple(float, float, float, float)
        :param right_target: wavelet paramters for right eyebrow
        :type right_target: tuple(float, float, float, float)
        :param steps: number of transition steps
        :type steps: int
        :param delay: delay between transition steps
        :type delay: float
        """
        # check if current face is polynomial based
        if not self.is_morphable:
            self._logger.error("Current face expression does not support morphing")
            raise ValueError("Current face expression does not support morphing")
        if "bitmap" in self._basis_functions:
            self._logger.error("Face contains images generated directly from a bitmap.")
            raise ValueError("Face contains images generated directly from a bitmap.")
        if "polynomial" in self._basis_functions:
            self._logger.error(
                "Missmatched basis functions, face contains both 'polynomial' "
                "and 'wavelet' based images"
            )
            raise ValueError(
                "Missmatched basis functions, face contains both 'polynomial' "
                "and 'wavelet' based images"
            )

        mouth_start = self._parameters["mouth"]
        left_start = self._parameters["left"]
        right_start = self._parameters["right"]

        # mouth
        self._logger.debug("Calculating mouth transitions")
        m1 = self._calculate_transition(mouth_start[0], m1_target)
        m2 = self._calculate_transition(mouth_start[1], m2_target)

        # left
        self._logger.debug("Calculating left eyebrow transitions")
        left = self._calculate_transition(left_start, left_target)

        # right
        self._logger.debug("Calculating right eyebrow transitions")
        right = self._calculate_transition(right_start, right_target)

        # show intermediate faces
        self._logger.debug("Displaying transition faces")
        for step in range(steps):
            self.send_wavelet_face(m1[step], m2[step], left[step], right[step])
            sleep(delay)
        # show final face
        self._logger.debug("Displaying target face")
        self.send_wavelet_face(m1_target, m2_target, left_target, right_target)

    def _calculate_transition(self, start, target, steps=3, padding=False):
        """
        Interpolates between start and target values.

        :param start: initial list
        :type start: list
        :param target: target list
        :type target: list
        :param steps: Number of interpolation steps
        :type steps: int
        :param padding: Number of interpolation steps
        :type padding: whether lists with missmatched sizes should be zero padded
        """
        # check if lengths match
        if len(start) != len(target):
            if padding:
                diff = len(start) - len(target)
                if diff < 0:
                    start += [0] * -diff
                else:
                    target += [0] * diff
            else:
                self._logger.error(
                    "Missmatched size (%i != %i)", len(start), len(target)
                )
                raise ValueError

        # calculate intermediate steps
        return [
            [
                start[i] + step * (target[i] - start[i]) / (steps + 1.0)
                for i in range(len(start))
            ]
            for step in range(1, steps + 1)
        ]
