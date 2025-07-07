from pathlib import Path
import platform
import sysconfig
import sys

# on Win we need to add the path to vendored DLLs manually, see
# https://github.com/adang1345/delvewheel/issues/62#issuecomment-2977988121
# the DLLs are installed to site-packages/ directly for some reason, see
# https://github.com/adang1345/delvewheel/issues/64
if platform.system().lower() == "windows":
    sys.path.append(sysconfig.get_paths()["purelib"])

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
