try:
    from .python_valhalla import Configure
except ModuleNotFoundError:
    from python_valhalla import Configure
from ._actor import Actor
from .buildtiles import BuildTiles
