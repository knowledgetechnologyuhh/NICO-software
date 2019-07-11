#!/usr/bin/env python
def wrapped_function():
    return None


# import the RemoteObject
from flaskcom.remote_object import RemoteObject

# wrap it around the function
# returns an object that can be used like the object initialized in the wrapped function,
# here: test_object = ComplexTestClass('hallo')
demo = RemoteObject(wrapped_function=wrapped_function,  # the function that initializes the remote object
                    # a virtualenv can loaded before exectuting the code in the remote terminal.
                    path_to_virtualenv="~/.NICO",
                    server="localhost",  # the remote object is running on another computer
                    # a working directory can be specified, which can be used to search for the code
                    original_working_directory=".",
                    keep_open=True,  # the remote object can be kept open, when the program is exectuted the next time, it will use the open remote object instead of creating a new one
                    time_out=-1,  # the time to wait for the remote terminal to start, -1 means forever
                    # if flaskcom is not inside the searchpath, set a path to a folder containing flaskcom
                    flaskcom_path="./",
                    debug=True,
                    start_server=False,
                    port=55100)  # keeps the terminal open even if an error occurs

demo.move_cube_coords((1, 0, 1), (0, 1, 1))
demo.move_cube_coords((1, 0, 0), (1, 1, 1))
demo.move_cube_coords((0, 1, 1), (1, 0, 0))
demo.move_cube_coords((1, 1, 1), (1, 0, 1))
