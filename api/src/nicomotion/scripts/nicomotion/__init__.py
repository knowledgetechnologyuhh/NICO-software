import sys
import os

try:
    from pyrep import PyRep  # required to prevent vrep launch issues
except (ImportError, SyntaxError):
    if (
        sys.version_info < (3,)
        or "COPPELIASIM_ROOT" not in os.environ
        or os.environ["COPPELIASIM_ROOT"] not in os.environ["LD_LIBRARY_PATH"]
        or os.environ["COPPELIASIM_ROOT"]
        not in os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"]
    ):
        pass  # pyrep is not Python 2 compatible
    else:
        raise
