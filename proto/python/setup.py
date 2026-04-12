"""Custom build step: generate Python code from .proto files using betterproto2.

Hooks into egg_info (the earliest setuptools step) so that the generated
packages exist before setuptools discovers them.

Uses grpcio-tools for protoc (bundled as a build dependency) and
betterproto2-compiler for the protoc plugin. No system protoc required.
"""

from pathlib import Path

from grpc_tools import protoc
from setuptools import setup
from setuptools.command.egg_info import egg_info

ROOT = Path(__file__).resolve().parent
PROTO_DIR = ROOT / "descriptors"  # symlink to proto/descriptors/
SRC_DIR = ROOT / "src"  # generated code lands here


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


class GenerateAndEggInfo(egg_info):
    """Generate proto code before setuptools collects package metadata."""

    def run(self):
        generate_proto()
        super().run()


setup(cmdclass={"egg_info": GenerateAndEggInfo})
