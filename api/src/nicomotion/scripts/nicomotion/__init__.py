import sys
import os

try:
    from pyrep import PyRep  # required to prevent vrep launch issues
except (ImportError, SyntaxError):
    if sys.version_info < (3,) or "VREP_ROOT" not in os.environ:
        pass  # pyrep is not Python 2 compatible
    else:
        raise
