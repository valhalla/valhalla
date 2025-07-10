from pathlib import Path

try:
    from ._valhalla import *
except ModuleNotFoundError:
    from _valhalla import *
from .actor import Actor
from .config import get_config, get_help

# if run from CMake, Docker or tests
try:
    from .__version__ import __version__
except ModuleNotFoundError:
    __version__ = "undefined"

PYVALHALLA_DIR = Path(__file__).parent.resolve()
