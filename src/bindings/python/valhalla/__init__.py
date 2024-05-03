try:
    from .python_valhalla import *
except ModuleNotFoundError:
    from python_valhalla import *

from .actor import Actor
from .config import get_config
