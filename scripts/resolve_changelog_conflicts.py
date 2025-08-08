#!/usr/bin/env python3
import re
import sys


def resolve_changelog_conflict(content):
    conflict = re.search(
        r'^(.*?)<<<<<<<.*?\n(.*?)\n=======\n(.*?)\n>>>>>>>([\s\S]*)$', content, re.DOTALL
    )
    if not conflict:
        return None

    before = conflict.group(1).strip()
    pr_entries = conflict.group(2).strip()
    master_entries = conflict.group(3).strip()
    after = conflict.group(4).strip()
    after = re.sub(r'^\s*master\s*$', '', after, flags=re.MULTILINE)

    combined = []
    seen = set()

    # First add master entries (newest changes)
    for entry in reversed(master_entries.split('\n')):
        entry = entry.strip()
        if entry and entry not in seen:
            combined.insert(0, f"   {entry}")
            seen.add(entry)

    # Then add PR entries (older changes)
    for entry in reversed(pr_entries.split('\n')):
        entry = entry.strip()
        if entry and entry not in seen:
            combined.insert(0, f"   {entry}")
            seen.add(entry)

    resolved = f"{before}\n" + "\n".join(combined) + (f"\n{after}" if after else "")
    return resolved


def main():
    if len(sys.argv) < 2:
        print("Usage: resolve_changelog_conflicts.py <changelog_path> [--dry-run]")
        sys.exit(1)

    changelog_path = sys.argv[1]
    dry_run = len(sys.argv) > 2 and sys.argv[2] == "--dry-run"

    with open(changelog_path, 'r') as f:
        content = f.read()

    resolved = resolve_changelog_conflict(content)
    if not resolved:
        print("No conflicts detected")
        sys.exit(0)

    if dry_run:
        print("Resolved changelog would be:")
        print(resolved)
        sys.exit(0)

    with open(changelog_path, 'w') as f:
        f.write(resolved)


if __name__ == "__main__":
    main()
