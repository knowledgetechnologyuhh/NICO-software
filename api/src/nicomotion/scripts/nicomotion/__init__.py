import sys

try:
    from pyrep import PyRep  # required to prevent vrep launch issues
except (ImportError, SyntaxError):
    if sys.version_info < (3,):
        pass  # pyrep is not Python 2 compatible
    else:
        raise
