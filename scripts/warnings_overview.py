#!/usr/bin/env python3

"""
This script analyzes a build log from Valhalla and pretty-prints warnings and their occurrences.
"""

from argparse import ArgumentParser
from collections import Counter, defaultdict
from pathlib import Path
import re
from typing import List, Optional

GCC = "gcc"
CLANG = "clang"
MSVC = "msvc"


description = "Parses a build log and prints a summary of observed warnings"

parser = ArgumentParser(description=description)
parser.add_argument(
    "--log-path", "-p", default=None, help="The full or relative path to the build log", type=Path
)
# Could probably auto-discover this from the first lines of the build log
parser.add_argument(
    "--compiler",
    "-cc",
    type=str,
    help="The compiler used for the build",
    choices=[
        GCC,
        # "clang",
        # "msvc"
    ],
    default=GCC,
)
# this is supposed to be used as "--list-example-files 10 -Wunused-parameter"
parser.add_argument(
    "--list-example-files",
    "-l",
    action="append",
    nargs=2,
    # only changes the --help displayed names
    metavar=("files_amount", "warning_id"),
    default=None,
    help="How many source file occurrences should be printed for a particular warning, e.g. '--list-example-files -1 -Wunused-parameter'. 'files_amount' = -1 prints all matching source paths.",
)


def main():
    args = parser.parse_args()

    # arguments
    log_path: Path = args.log_path
    compiler: str = args.compiler.lower()
    list_parameters: Optional[List[List[str]]] = args.list_example_files or []

    print(
        f"[INFO] Analyzing build log {log_path} for compiler {compiler} with details for: {'None' if not list_parameters else ''}"
    )
    if list_parameters:
        for files_amount, warning_id in list_parameters:
            print(f"  {warning_id}: max {'all' if files_amount != -1 else files_amount} source paths")

    warnings_counter = Counter()
    warnings_files_requested = defaultdict(int)
    warnings_files_lines = defaultdict(list)
    with log_path.open() as log_f:
        for files_amount, warning_id in list_parameters:
            warnings_files_requested[warning_id] = int(files_amount)

        if compiler == GCC:
            for line_idx, line in enumerate(log_f.readlines()):
                # match "[-W" for each line and capture the whole [-W...] block
                if match := re.search(r'(\[-W[^\]]+\])', line):
                    # remove the brackets
                    warning_id = match.group(1)[1:-1]
                    warnings_counter[warning_id] += 1
                    # append the whole line for inspection
                    if warnings_files_requested[warning_id]:
                        warnings_files_lines[warning_id].append(line)

    # log the results
    if warnings_counter:
        print("\n=== Warning counts ===")
        for warn, count in warnings_counter.most_common():
            print(f"{warn}: {count}")

        if warnings_files_lines:
            print("\n=== Messages per Warning ===")
            for warning_id, lines in warnings_files_lines.items():
                req_warning_amount = warnings_files_requested[warning_id]
                # if someone set 0 for some reason
                if not req_warning_amount:
                    continue

                print(
                    f"\n{warning_id}: ({warnings_counter[warning_id]} total, showing {warnings_files_requested[warning_id]})"
                )

                line: str
                for line_idx, line in enumerate(lines):
                    if req_warning_amount == -1 or line_idx < warnings_files_requested[warning_id]:
                        print(f"  {line.strip()}")
                        continue
                    break


if __name__ == "__main__":
    main()
