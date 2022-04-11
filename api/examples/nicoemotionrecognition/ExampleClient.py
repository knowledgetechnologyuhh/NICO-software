from nicoemotionrecognition.EmotionRecognition import EmotionRecognition
from nicovision.VideoDevice import VideoDevice


def example_client():
    print
    print("----------------------")
    print("running example_client")

    # create Emotion object (make sure server is running in another terminal)
    emotion = EmotionRecognition()

    # video device callback
    def _callback(rval, frame):
        if rval:
            emotion.send_image(frame)
            if emotion.face_detected:
                print("Face detected at: {}".format(emotion.get_face_center()))
                print(
                    "Highest matching emotion: {}".format(
                        emotion.get_highest_matching_emotion()
                    )
                )
                print("Individual Scores: {}".format(emotion.get_categorical_data()))

    # initialize video device
    # device_name = VideoDevice.autodetect_nicoeyes()[0]
    # device = VideoDevice.from_device(device_name)
    device = VideoDevice(0)
    device.add_callback(_callback)
    device.open()

    # stop program from terminating
    input("Press <enter> to stop")

    # cleanup
    emotion.close()
    device.close()


if __name__ == "__main__":
    example_client()
