from nicoface.CapacitiveSensors import CapacitiveSensors
from nicoface.FaceExpression import faceExpression
import logging
from threading import Thread
import time


def printCapacitiveReadings(capacitive_sensor):
    while running:
        readings = capacitive_sensor.getCapacitiveReadings()
        print("\r{} pads. Values:".format(len(readings))),
        print(readings)

        # if readings need to be recallibrated, call
        # 		myFace.recallibrateCapacitivePads().
        # this should not normally be needed because a callibration is automatically performed
        # when the face controller is powered on.


if __name__ == "__main__":
    print("Getting capacitive touch readings from Nico face. Press Ctrl+C to terminate")

    logging.basicConfig(level=logging.INFO)  # needed for logger in FaceExpression
    capacitive_sensor = CapacitiveSensors()
    face = faceExpression()
    running = True
    t = Thread(target=printCapacitiveReadings, args=(capacitive_sensor,))
    t.start()

    face.sendFaceExpression("happiness")

    time.sleep(1)

    face.sendFaceExpression("anger")

    time.sleep(1)

    face.sendFaceExpression("sadness")

    time.sleep(1)

    face.sendFaceExpression("happiness")

    running = False

    t.join()
