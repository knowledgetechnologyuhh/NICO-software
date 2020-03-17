from nicoface.CapacitiveSensors import CapacitiveSensors
import logging


def printCapacitiveReadings():

    readings = myFace.getCapacitiveReadings()
    print("\r{} pads. Values:".format(len(readings))),
    print(readings)

    # if readings need to be recallibrated, call
    # 		myFace.recallibrateCapacitivePads().
    # this should not normally be needed because a callibration is automatically performed
    # when the face controller is powered on.


if __name__ == "__main__":
    print("Getting capacitive touch readings from Nico face. Press Ctrl+C to terminate")

    logging.basicConfig(level=logging.INFO)  # needed for logger in FaceExpression
    myFace = CapacitiveSensors()

    while 1:
        # print("iteration {}".format(b))
        printCapacitiveReadings()
