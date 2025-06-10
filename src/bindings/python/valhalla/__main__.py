import subprocess
import sys
from typing import Optional

from ._scripts import PYVALHALLA_BIN_DIR, run

try:
    from .__version__ import version as __version__
except ImportError:
    __version__ = "undefined"

PRINT_BIN_PATH = "print_bin_path"


def print_help():
    # left-align executable names
    exe_names = [p.name for p in PYVALHALLA_BIN_DIR.iterdir()]
    fixed_width = len(max(exe_names, key=lambda x: len(x)))

    # TODO: we also want the git commit to show here, probably written to __version__.py by setup.py's BDistWheelCommand
    print(
        F"""
valhalla Python package {__version__}

pyvalhalla package provides a CLI to run Valhalla C++ executables. Arguments are simply passed on as they are.
One notable exception is --quiet/-q, which will forward the C++ executable's stdout to /dev/null.
'python -m valhalla --help' will print this help.

Example usage: 'python -m valhalla --quiet valhalla_build_tiles -c valhalla.json -s initialize -e build test.pbf'

Available commands (in {PYVALHALLA_BIN_DIR}):
\t{PRINT_BIN_PATH:<{fixed_width}} - Print the absolute path of directory containing the C++ executables
"""
    )


def main():
    if len(sys.argv) < 2:
        print("[FATAL] command needs arguments, see 'python -m valhalla --help': \n")
        print_help()
        sys.exit(1)

    prog_or_opt = sys.argv[1]

    if prog_or_opt in ("--help", "-h"):
        print_help()
        sys.exit(0)
    elif prog_or_opt == PRINT_BIN_PATH:
        # useful when another script wants to run the executables directly for some reason
        print(PYVALHALLA_BIN_DIR)
    else:
        exc: Optional[Exception] = None
        try:
            run(from_main=True)
        except (FileNotFoundError, subprocess.CalledProcessError) as e:
            exc = e
        finally:
            if not exc:
                sys.exit(0)

            if isinstance(exc, subprocess.CalledProcessError):
                print(f"Failed calling command '{exc.cmd}'\n")

            print_help()
            print("Sub-command failed with:\n")
            print(exc, file=sys.stderr)

            sys.exit(1)


if __name__ == "__main__":
    main()
