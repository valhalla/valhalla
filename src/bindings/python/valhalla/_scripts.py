from pathlib import Path
import platform
from shutil import which
import subprocess
import sys

from . import PYVALHALLA_DIR, VALHALLA_PYTHON_PACKAGE


PYVALHALLA_BIN_DIR = PYVALHALLA_DIR.joinpath("bin").resolve()
IS_WIN = platform.system().lower() == "windows"

# on the filesystem it's not pyvalhalla-weekly, but pyvalhalla_weekly
mpath = VALHALLA_PYTHON_PACKAGE.replace("-", "_")
VENDORED_LIB_DIR = Path(__file__).parent.parent.joinpath(mpath + ".libs").resolve()


def run(from_main=False) -> None:
    """
    Parses the command line arguments and runs the Valhalla executables with the
    provided arguments. Note, by default we assume this is not being called by
    __main__.py via e.g. 'python -m valhalla ...', but directly with e.g.d
    'valhalla_build_tiles -h'. sys.argv relates to different executables for
    both scenarios.

    :param from_main: We parse sys.argv differently if it's called from __main__.py
    """
    prog = sys.argv[1] if from_main else Path(sys.argv[0]).name
    prog_args = sys.argv[2:] if from_main else sys.argv[1:]

    if not which(prog):
        raise FileNotFoundError(f"Can't find executable at {prog}")
    prog_path = PYVALHALLA_BIN_DIR.joinpath(prog + (".exe" if IS_WIN and from_main else "")).resolve()

    print(f"[INFO] Running {prog_path} with args: {prog_args}...")

    # --quiet/-q is our option, don't pass it to the executables
    is_quiet = False
    for arg in prog_args:
        if arg not in ("--quiet", "-q"):
            continue
        prog_args.pop(prog_args.index(arg))
        is_quiet = True

    # recommended way to call exe
    proc = subprocess.run(
        [str(prog_path)] + prog_args,
        stderr=sys.stderr,
        stdout=subprocess.DEVNULL if is_quiet else sys.stdout,
        # on Win we need to add the path to vendored DLLs manually, see
        # https://github.com/adang1345/delvewheel/issues/62#issuecomment-2977988121
        env=(dict(PATH=str(VENDORED_LIB_DIR)) if IS_WIN else None),
    )

    # raises CalledProcessError if not successful
    proc.check_returncode()
