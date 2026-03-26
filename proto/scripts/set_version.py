#!/usr/bin/env python3

import subprocess
import sys
import datetime
import re
from pathlib import Path
import json
import tomllib


ROOT = Path(__file__).resolve().parents[1]
VALHALLA_DIR = ROOT.parent

PYPROJECT = ROOT / "python" / "pyproject.toml"
TS_PACKAGE_JSON = ROOT / "ts" / "package.json"

SEMVER_RE = re.compile(r"^\d+\.\d+\.\d+$")


def git(*args: str, cwd: Path) -> str:
    return subprocess.check_output(
        ["git", *args],
        cwd=cwd,
        text=True,
    ).strip()


def latest_tag() -> str:
    tag = git("describe", "--tags", "--abbrev=0", cwd=VALHALLA_DIR)
    if not SEMVER_RE.match(tag):
        raise RuntimeError(f"Latest tag '{tag}' is not strict X.Y.Z format")
    return tag


def head_commit() -> str:
    return git("rev-parse", "HEAD", cwd=VALHALLA_DIR)


def tag_commit(tag: str) -> str:
    return git("rev-list", "-n", "1", tag, cwd=VALHALLA_DIR)


def short_sha() -> str:
    return git("rev-parse", "--short", "HEAD", cwd=VALHALLA_DIR)


def compute_version(separator: str) -> str:
    tag = latest_tag()
    head = head_commit()
    tag_sha = tag_commit(tag)

    if head == tag_sha:
        return tag

    today = datetime.date.today().strftime("%Y%m%d")
    return f"{tag}{separator}dev{today}"


def write_version_typescript(version: str) -> None:
    with open(TS_PACKAGE_JSON, "r") as f:
        data = json.load(f)
    data["version"] = version
    with open(TS_PACKAGE_JSON, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Set ts/package.json version to {version}")


def write_version_python(version: str) -> None:
    import tomli_w

    with open(PYPROJECT, "rb") as f:
        data = tomllib.load(f)

    data["project"]["version"] = version

    with open(PYPROJECT, "wb") as f:
        tomli_w.dump(data, f)
    print(f"Set python/pyproject.toml version to {version}")


def main() -> None:
    lang = ""
    if len(sys.argv) == 2:
        lang = sys.argv[1].lower()

    if lang == "ts" or lang == "":
        version = compute_version("-")
        write_version_typescript(version)
    if lang == "py" or lang == "":
        version = compute_version(".")
        write_version_python(version)


if __name__ == "__main__":
    main()
