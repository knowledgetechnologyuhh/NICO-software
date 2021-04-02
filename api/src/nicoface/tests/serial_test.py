import os
import pty
import random
import string
import threading
import unittest

from nicoface.SerialConnectionManager import SerialDevice


def fake_serial(port):
    while True:
        res = os.read(port, 256)
        os.write(port, res)


class SerialTest(unittest.TestCase):
    def setUp(self):
        master, slave = pty.openpty()
        ser_name = os.ttyname(slave)
        self.ser = SerialDevice(ser_name, 115200, 0.016)
        thread = threading.Thread(target=fake_serial, args=[master])
        thread.daemon = True
        thread.start()

    def tearDown(self):
        del self.ser

    def test_connection(self):
        response = self.ser.send("test")
        self.assertEqual(response, b"test")

    def test_connection_random(self):
        message = "".join(random.choice(string.ascii_letters) for _ in range(8))
        response = self.ser.send(message)
        self.assertEqual(response, message.encode("utf-8"))


if __name__ == "__main__":
    unittest.main()
