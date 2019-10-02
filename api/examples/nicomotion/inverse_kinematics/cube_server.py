#!/usr/bin/env python
from cubes_cached import CubeDemo
from flaskcom.server import ServerWrapper

if __name__ == '__main__':
    cube_object = CubeDemo()

    cube_object = ServerWrapper(cube_object, 55100, complex_datatypes=True)
    cube_object.ServerWrapper_start()
