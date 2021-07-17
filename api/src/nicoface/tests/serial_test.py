import os
import pty
import random
import string
import threading
import unittest

from nicoface.SerialConnectionManager import SerialDevice


def fake_serial(port):
    """Mock serial that returns the data it reads"""
    while True:
        res = os.read(port, 256)
        os.write(port, res)


class SerialTest(unittest.TestCase):
    def setUp(self):
        # create pseudoterminal to simulate serial device
        master, slave = pty.openpty()
        ser_name = os.ttyname(slave)
        # open serial connection
        self.ser = SerialDevice(ser_name, 115200, 0.016)
        # start thread that runs fake serial in background
        thread = threading.Thread(target=fake_serial, args=[master])
        thread.daemon = True
        thread.start()

    def tearDown(self):
        # close serial
        del self.ser

    def test_connection(self):
        """Test if send message is returned successfully"""
        response = self.ser.send("test")
        self.assertEqual(response, b"test")

    def test_connection_random(self):
        """Tests connection with a random string"""
        message = "".join(random.choice(string.ascii_letters) for _ in range(8))
        response = self.ser.send(message)
        self.assertEqual(response, message.encode("utf-8"))


if __name__ == "__main__":
    unittest.main()
