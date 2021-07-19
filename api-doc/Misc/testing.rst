Testing
=======
To verify whether the installation was successful, you can execute the following test script:

.. code-block:: bash

    source NICO-test.bash


This script will run a series of tests for the ``nicoaudio``, ``nicoface`` and
``nicomotion`` modules (see :ref:`below<Test Coverage>`). Note that it will only test functions independant of
external hardware. Tests for the ``nicomotion`` require ``pyrep``.

If you only want to test a specific module, you can also run ``pytest`` directly
in the corresponding test directory, e.g. ``nicoaudio``:

.. code-block:: bash

    pytest -v src/nicoaudio/tests

.. _Test Coverage:

Test Coverage
-------------

Here is a list of all the existing tests with a short description

+---------------+--------------------+-----------------------------------------+
| Module        | Test               | Description                             |
+===============+====================+=========================================+
| nicoaudio     | audioplayer_test   | Tests basic playback functionality as   |
|               |                    | well as manipulation of duration,       |
|               |                    | volume, pitch and speed.                |
+---------------+--------------------+-----------------------------------------+
| nicoface      | face_test          | Tests different face changing methods   |
|               |                    | on a virtual face. It does not require  |
|               |                    | the real robot's face to be connected.  |
|               +--------------------+-----------------------------------------+
|               | serial_test        | Uses a virtual serial connection to     |
|               |                    | test if messages can be send and        |
|               |                    | received correctly.                     |
+---------------+--------------------+-----------------------------------------+
| nicomotion    | pyrep_motion_test  | Simulates NICO with pyrep to test if    |
|               |                    | angles can be adjusted correctly via    |
|               |                    | ``Motion``.                             |
|               |                    | This does not require the real robot to |
|               |                    | be connected.                           |
|               +--------------------+-----------------------------------------+
|               | visualizer_test    | Tests if angles and target position of  |
|               |                    | ``Visualizer`` can be set properly.     |
+---------------+--------------------+-----------------------------------------+
