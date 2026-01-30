#!/usr/bin/env python3
"""
Merge two Valhalla JSON config files.

- Keys only in new JSON: added with new value
- Keys only in old JSON: kept (or removed with -R flag)
- Keys in both: old value is preserved
- Nested objects are merged recursively
"""

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
import sys
from typing import Any


@dataclass
class MergeReport:
    """Tracks changes during merge."""

    added: list[str] = field(default_factory=list)
    removed: list[str] = field(default_factory=list)

    def print_report(self):
        if self.added:
            print("\nAdded (from new config):", file=sys.stderr)
            for path in sorted(self.added):
                print(f"  + {path}", file=sys.stderr)

        if self.removed:
            print("\nRemoved (not in new config):", file=sys.stderr)
            for path in sorted(self.removed):
                print(f"  - {path}", file=sys.stderr)

        if not (self.added or self.removed):
            print("\nNo differences found.", file=sys.stderr)


def merge_json(old: Any, new: Any, report: MergeReport, remove_old: bool, path: str = "") -> Any:
    """
    Recursively merge two JSON values.

    Existing values in 'old' are preserved.
    Keys only in 'old' are kept unless remove_old is True.
    """
    # If new is not a dict, keep old value (preserve existing)
    if not isinstance(new, dict):
        return old

    # Both are dicts - merge recursively, preserving old key order
    result = {}

    # First, process keys from old (preserves original order)
    for key in old:
        key_path = f"{path}.{key}" if path else key
        if key in new:
            # Key exists in both - recursively merge
            result[key] = merge_json(old[key], new[key], report, remove_old, key_path)
        elif remove_old:
            # Key only in old and remove_old is set - drop it
            report.removed.append(key_path)
        else:
            # Key only in old - keep it
            result[key] = old[key]

    # Then, add new keys that don't exist in old (at the end)
    for key in new:
        if key not in old:
            key_path = f"{path}.{key}" if path else key
            result[key] = new[key]
            report.added.append(key_path)

    return result


parser = argparse.ArgumentParser(
    description="Merge old config with new config. "
    "Existing values are preserved. "
    "New keys are added with their default values."
)
parser.add_argument(
    "old",
    help="Path to existing config file, e.g. valhalla.json",
    type=Path,
)
parser.add_argument(
    "new",
    help="Path to new config file, e.g. new_valhalla.json",
    type=Path,
)
parser.add_argument(
    "-o",
    "--output",
    help="Output file path (default: stdout)",
    type=Path,
)
parser.add_argument(
    "-R",
    "--remove-old",
    help="Removes JSON keys which don't exist anymore in new config.",
    action="store_true",
)
parser.add_argument(
    "-r",
    "--report",
    help="Will print which ones were added and/or removed",
    action="store_true",
)


def main():
    args = parser.parse_args()

    # open old config
    try:
        with args.old.open("r") as f:
            old_data = json.load(f)
    except FileNotFoundError:
        print(f"Error: Old config file not found: {args.old}", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in old config: {e}", file=sys.stderr)
        sys.exit(1)

    # open new config
    try:
        with args.new.open("r") as f:
            new_data = json.load(f)
    except FileNotFoundError:
        print(f"Error: New config file not found: {args.new}", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON in new config: {e}", file=sys.stderr)
        sys.exit(1)

    report = MergeReport()
    merged = merge_json(old_data, new_data, report, args.remove_old)

    output_json = json.dumps(merged, indent=2)

    if args.output:
        with args.output.open("w") as f:
            f.write(output_json)
            f.write("\n")
        print(f"Merged config written to: {args.output}", file=sys.stderr)
    else:
        print(output_json)

    if args.report:
        report.print_report()


if __name__ == "__main__":
    main()
