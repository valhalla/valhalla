from pathlib import Path

try:
    from ._valhalla import VALHALLA_PRINT_VERSION, VALHALLA_PYTHON_PACKAGE
except ModuleNotFoundError:
    from _valhalla import VALHALLA_PRINT_VERSION, VALHALLA_PYTHON_PACKAGE
from .actor import Actor
from .config import get_config, get_help

# if run from CMake, Docker or test
try:
    from .__version__ import __version__

    # extend with version modifier (so far the git hash)
    if (idx := VALHALLA_PRINT_VERSION.find("-")) != -1:
        __version__ = __version__ + VALHALLA_PRINT_VERSION[idx:]
except ModuleNotFoundError:
    __version__ = "undefined"

PYVALHALLA_DIR = Path(__file__).parent.resolve()

__all__ = ["Actor", "get_config", "get_help", "__version__"]
