"""Custom build step: generate Python code from .proto files using betterproto2.

Hooks into egg_info (the earliest setuptools step) so that the generated
packages exist before setuptools discovers them.

Uses grpcio-tools for protoc (bundled as a build dependency) and
betterproto2-compiler for the protoc plugin. No system protoc required.
"""

import subprocess
from pathlib import Path

from grpc_tools import protoc
from setuptools import setup
from setuptools.command.egg_info import egg_info

ROOT = Path(__file__).resolve().parent
PROTO_DIR = ROOT / "descriptors"  # symlink to proto/descriptors/
SRC_DIR = ROOT / "src"  # generated code lands here
REPO_ROOT = ROOT / ".." / ".."


def generate_proto():
    """Run protoc with the betterproto2 plugin to generate Python code."""
    SRC_DIR.mkdir(exist_ok=True)
    protos = sorted(PROTO_DIR.glob("*.proto"))
    if not protos:
        raise FileNotFoundError(f"No .proto files found in {PROTO_DIR}")
    rc = protoc.main(
        [
            "protoc",
            f"--python_betterproto2_out={SRC_DIR}",
            f"-I{PROTO_DIR}",
            *[str(p) for p in protos],
        ]
    )
    if rc != 0:
        raise RuntimeError(f"protoc failed with exit code {rc}")

    # "python -m build" needs a COMMIT file, since it doesn't run
    # egg_info in the source tree, but in the isolated build tree
    # "pip wheel" runs egg_info in the source tree with git available
    commit = "unknown"
    commit_file = ROOT / "COMMIT"
    if commit_file.exists():
        commit = commit_file.read_text().strip()
    else:
        try:
            commit = subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"],
                cwd=REPO_ROOT,
                text=True,
            ).strip()
        except (subprocess.CalledProcessError, FileNotFoundError):
            pass
    (SRC_DIR / "_upstream.py").write_text(f'VALHALLA_COMMIT = "{commit}"\n')


class GenerateAndEggInfo(egg_info):
    """Generate proto code before setuptools collects package metadata."""

    def run(self):
        generate_proto()
        super().run()


setup(cmdclass={"egg_info": GenerateAndEggInfo})
