from pathlib import Path

try:
    from ._valhalla import *
except ModuleNotFoundError:
    from _valhalla import *
from .actor import Actor
from .config import get_config, get_help
from .__version__ import __version__

PYVALHALLA_DIR = Path(__file__).parent.resolve()
