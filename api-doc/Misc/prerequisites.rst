Prerequisites
=============

Most prerequesites are automatically installed by sourcing ``NICO-setup.bash`` or
``NICO-python3.bash``. However some environment variables and permissions need to be set and apt packages to be installed for all the libraries to work.

PyRep
-----

PyRep, which can be used to control and interact with CoppeliaSim, is
automatically installed by the setup, but requires `COPPELIASIM_ROOT` to point
to the folder where CoppeliaSim is located. To do so, edit the path and add
this to your ``.bashrc``:

.. code-block:: bash

  export COPPELIASIM_ROOT=EDIT/ME/PATH/TO/COPPELIASIM/INSTALL/DIR

Additionally, the `COPPELIASIM_ROOT` has to be added to `LD_LIBRARY_PATH` and `QT_QPA_PLATFORM_PLUGIN_PATH` to run PyRep. Since these can cause conflicts with other programms, a bash script is provided that can be sourced whenever PyRep is needed:

.. code-block:: bash

  source pyrep_env.bash


Alternatively, add the following to your `.bashrc`
(**note that this can cause conflicts with other programms**):

.. code-block:: bash

  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
  export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT

See https://github.com/stepjam/PyRep for more information about PyRep.


nicoaudio
---------

`PyAudio` requires `portaudio`:

.. code-block:: bash

  sudo apt-get install portaudio19-dev

`PyDub` requires `ffmpeg` to process non-wav files:

.. code-block:: bash

  sudo apt-get install ffmpeg

TextToSpeech:

The `nicoaudio.TextToSpeech` uses `pico2wave` as fallback (e.g if there is no internet):

.. code-block:: bash

  sudo apt-get install pico2wave

It can also use a locally hosted `MozillaTTS server <https://github.com/mozilla/TTS/tree/master/server>`_ (note that MozillaTTS is still in developement)

nicomotion
----------
To open a serial connection, you need permissions for the port. This can be
solved by adding the user to the dialout group:

.. code-block:: bash

  sudo adduser $USER dialout

otherwise you need to manually set the permissions each time you open a shell:

.. code-block:: bash

  sudo chmod 777 /dev/ttyACM*

`nicomotion.Motion` also tries to adjust port latency using `setserial`:

.. code-block:: bash

  sudo apt-get install setserial


nicovision
----------

v4l2-ctl:

.. code-block:: bash

  sudo apt-get install v4l-utils


nicoros
-------

For informations on ROS see https://www.ros.org/

nicotouch
---------
To open a serial connection, you need permissions for the port. This can be
solved by adding the user to the dialout group:

.. code-block:: bash

  sudo adduser $USER dialout

otherwise you need to manually set the permissions each time you open a shell:

.. code-block:: bash

  sudo chmod 777 /dev/ttyACM*
