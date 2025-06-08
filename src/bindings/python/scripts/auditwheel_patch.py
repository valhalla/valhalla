#!/usr/bin/env python3

"""
This script patches auditwheel to allow libraries such as libz & libexpat to be vendored into the wheel.

See discussion (and related discussions):
https://github.com/valhalla/valhalla/pull/5292#issuecomment-2953151580
"""

from argparse import ArgumentParser
import json
from pathlib import Path
import sys
from typing import List

try:
    import auditwheel
except ImportError:
    print(
        f"[FATAL] Can't find 'auditwheel' package in current Python environment at {Path(sys.executable).parent.parent}.",
        file=sys.stderr,
    )
    sys.exit(1)

# does the required JSON exist?
AUDITWHEEL_POLICY_JSON = Path(auditwheel.__file__).parent.joinpath("policy", "manylinux-policy.json")
if not AUDITWHEEL_POLICY_JSON.exists():
    print(f"[FATAL] Can't find source to patch: {AUDITWHEEL_POLICY_JSON}", file=sys.stderr)

description = "Builds a tar extract from the tiles in mjolnir.tile_dir to the path specified in mjolnir.tile_extract."

parser = ArgumentParser(description=description)
parser.add_argument("SONAMES", help="The .so names to remove from the auditwheel policy.", nargs="+")
parser.add_argument(
    "-p", "--policy", help="The 'manylinux' policy name", type=str, default="manylinux_2_28"
)


def main():
    args = parser.parse_args()

    print(f"[INFO] Modifying {AUDITWHEEL_POLICY_JSON}...")

    with AUDITWHEEL_POLICY_JSON.open() as f:
        policy_source = json.load(f)

        # even though we only use 1 policy, we need to remove the libs from all, otherwise
        # auditwheel says PEP600 is violated
        for policy in policy_source:
            lib_whitelist: List[str] = policy["lib_whitelist"]
            for lib_name in args.SONAMES:
                if lib_name not in lib_whitelist:
                    print(
                        f"[WARN] Can't find {lib_name} in {policy['name']}.lib_whitelist",
                        file=sys.stderr,
                    )
                    continue
                lib_whitelist.pop(lib_whitelist.index(lib_name))

    with AUDITWHEEL_POLICY_JSON.open("w") as f:
        json.dump(policy_source, f)


if __name__ == "__main__":
    main()
