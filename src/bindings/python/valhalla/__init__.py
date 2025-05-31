try:
    from ._valhalla import *
except ModuleNotFoundError:
    from _valhalla import *

from .actor import Actor
from .config import get_config
