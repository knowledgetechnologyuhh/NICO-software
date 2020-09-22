.. NICO documentation master file

Welcome to the NICO documentation
=================================

This is the documentation of the high-level API for the NICO robot.

To simplify setup, a bash-script has been provided which you can source.

Python 2.7:

.. code-block:: bash

   source api/NICO-setup.bash

Python 3:

.. code-block:: bash

   source api/NICO-python3.bash

Either setup will generate an activation script that can be sourced
to use the api without reinstalling or updating packages:

.. code-block:: bash

   source api/activate.bash

See :doc:`/Misc/prerequisites` for additional instructions.

Table Of Contents
-----------------

.. toctree::
   :maxdepth: 3
   :glob:

   Misc/*
   Core/*
   Ros/*
