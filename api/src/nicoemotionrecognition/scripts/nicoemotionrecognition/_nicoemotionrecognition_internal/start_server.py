#!/usr/bin/env python
import argparse
import gc


def start_emotion_recognition_server(gui):

    # this function is run by the remote server
    # put all code inside that is needed to initialize your remote object
    def wrapped_function():
        from EmotionRecognitionBackend import EmotionRecognitionBackend

        emotion_object = EmotionRecognitionBackend(gui)

        return emotion_object

    from flaskcom.remote_object import RemoteObject
    from flaskcom.remote_object import VERBOSITY_ERROR

    # creates the server object from the wrapped_function
    test_server = RemoteObject(
        # the function that initializes the remote object
        wrapped_function=wrapped_function,
        # path_to_virtualenv = "./env3", #a virtualenv can loaded before exectuting the code in the remote terminal.
        # must be localhost. this script can only be started when started from the server
        server="localhost",
        # server = "localhost", #the remote object is on this computer
        port=50000,  # a port needs to be specified
        # forces the code to be run in the terminal it was started from
        new_terminal_window=False,
        # a working directory can be specified, which can be used to search for the code
        original_working_directory=".",
        keep_open=False,  # the terminal is not kept open. if True, the server will run in the background till the terminal is closed
        time_out=-1,  # the time to wait for the remote terminal to start, -1 means forever
        wait_for_errors=0,  # dont wait for the server to be started
        # makes flaskcom more verbose for debugging, can be changed to VERBOSITY_ERROR to be less verbose
        verbosity_level=VERBOSITY_ERROR,
        username="admin",  # the admin username for the remote object
        password="mypassword1",
    )  # the admin password for the remote object
    # the remote server was created
    print()
    print("\033[32mReady to receive images\033[0;0m")
    while test_server._running:
        gc.collect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=("Starts backend for emotion recognition model with or without GUI")
    )

    parser.add_argument(
        "--disable-gui", dest="gui", action="store_false", help="Disables the GUI."
    )

    args = parser.parse_args()
    start_emotion_recognition_server(args.gui)
