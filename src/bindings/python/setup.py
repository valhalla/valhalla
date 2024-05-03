import json
import logging
import os
from pathlib import Path
import platform

from pybind11.setup_helpers import Pybind11Extension
from setuptools import find_packages, setup


THIS_DIR = Path(__file__).parent.resolve()

include_dirs = [
    str(THIS_DIR.joinpath("include", "common")),
    # some includes are referencing like <baldr/..> instead of <valhalla/baldr/..>
    str(THIS_DIR.joinpath("include", "common", "valhalla")),
    str(THIS_DIR.joinpath("include", platform.system().lower())),
]
library_dirs = [str(THIS_DIR.joinpath("lib", platform.system().lower()))]
libraries = list()
extra_link_args = list()
extra_compile_args = list()
extra_objects = list()

if platform.system() == "Windows":
    libraries.extend(["libprotobuf-lite", "valhalla", "libcurl", "zlib", "Ws2_32", "ole32", "Shell32"])
    extra_compile_args.extend(["-DNOMINMAX", "-DWIN32_LEAN_AND_MEAN", "-DNOGDI"])
else:
    # protobuf_lib = str(THIS_DIR.joinpath("lib", platform.system().lower(), "libprotobuf-lite.a"))
    libraries.extend(["protobuf-lite", "valhalla", "curl", "z"])
    extra_link_args.extend(["-lvalhalla", "-lprotobuf-lite", "-lcurl", "-lz"])
    # extra_objects.append(protobuf_lib)

# do conan dependency resolution
# locally there will be 2 conanbuildinfo.json, one here and one in ./upstream/conan_build
conanfiles = Path(__file__).parent.resolve().rglob("conanbuildinfo.json")
conanfiles = tuple(filter(lambda p: p.parent.parent.name != "upstream", conanfiles))
if conanfiles:
    logging.info("Using conan to resolve dependencies.")
    with conanfiles[0].open() as f:
        # it's just header-only boost so far..
        include_dirs.extend(json.load(f)["dependencies"][0]["include_paths"])
else:
    logging.warning(
        "Conan not installed and/or no conan build detected. Assuming dependencies are installed."
    )

ext_modules = [
    Pybind11Extension(
        "_valhalla",
        [os.path.join("valhalla", "_valhalla.cc")],
        cxx_std=17,
        include_pybind11=False,  # use submodule'd pybind11
        library_dirs=library_dirs,
        include_dirs=include_dirs,
        extra_link_args=extra_link_args,
        extra_compile_args=extra_compile_args,
        extra_objects=extra_objects,
        libraries=libraries,
    ),
]

# open README.md for PyPI
with open(os.path.join(THIS_DIR, "README.md"), encoding="utf-8") as f:
    long_description = "\n" + f.read()

setup(
    name="pyvalhalla",
    description="High-level bindings to the Valhalla C++ library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Nils Nolde",
    author_email="nils@gis-ops.com",
    packages=find_packages(exclude=["tests", "*.tests", "*.tests.*", "tests.*"]),
    python_requires=">=3.7.0",
    url="https://github.com/gis-ops/pyvalhalla",
    ext_package="valhalla",
    ext_modules=ext_modules,
    zip_safe=False,
    classifiers=[
        "License :: OSI Approved :: GNU General Public License v2 or later (GPLv2+)",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: Implementation :: CPython",
    ],
)
