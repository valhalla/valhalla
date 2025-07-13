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

    try:
        from .__moduleinfo__ import __version_modifier__

        __version__ = f"{__version__}-{__version_modifier__}" if __version_modifier__ else __version__
    except ModuleNotFoundError:
        pass
except ModuleNotFoundError:
    __version__ = "undefined"

PYVALHALLA_DIR = Path(__file__).parent.resolve()
