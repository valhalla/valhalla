import subprocess
import sys

from ._scripts import PYVALHALLA_BIN_DIR, run

PRINT_BIN_PATH = "print_bin_path"


def main():
    print(f"Original sys.argv:\n{sys.argv}")
    prog = sys.argv[1]

    try:
        if prog == "help":
            print(
                "\npyvalhalla package provides a CLI to run Valhalla C++ executables. Arguments are simply passed on as they are."
            )
            print(
                "One notable exception is --quiet/-q, which will forward the C++ executable's stdout to /dev/null."
            )
            print(
                "\nExample usage: 'python -m valhalla --quiet valhalla_build_tiles -c valhalla.json -s initialize -e build test.pbf'"
            )
            print("\nAvailable commands:")
            # left-align executable names
            exe_names = [p.name for p in PYVALHALLA_BIN_DIR.iterdir()]
            fixed_width = len(max(exe_names, key=lambda x: len(x)))

            print(
                f"\t{PRINT_BIN_PATH:<{fixed_width}} - Print the absolute path of directory containing the C++ executables"
            )
            for command in exe_names:
                print(
                    f"\t{command:<{fixed_width}} - see usage with 'python -m pyvalhalla {command} --help' "
                )
            sys.exit(0)

        elif prog == PRINT_BIN_PATH:
            print(PYVALHALLA_BIN_DIR)
        else:
            # run(prog, sys.argv[2:])
            run(from_main=True)

    except subprocess.CalledProcessError as e:
        print(f"Failed calling command '{e.cmd}'")
        print(e, file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError as e:
        print(e, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
