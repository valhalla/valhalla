import os
from pathlib import Path
import platform
import warnings

from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup


THIS_DIR = Path(__file__).parent.resolve()

include_dirs = [
    str(THIS_DIR.joinpath("third_party", "date", "include")),
    str(THIS_DIR.joinpath("third_party", "pybind11", "include")),
    str(THIS_DIR.joinpath("third_party", "rapidjson", "include")),
    str(THIS_DIR.joinpath("valhalla")),
    str(THIS_DIR),
]
library_dirs = ["/usr/local/lib", "/usr/local/lib64"]
libraries = list()
extra_link_args = list()
extra_compile_args = list()

# determine the directories for compiling
if platform.system().lower() == "darwin":
    include_dirs.append("/opt/homebrew/include")
    library_dirs.append("/opt/homebrew/lib")
elif platform.system().lower() == "windows":
    # this is set in GHA to the vcpkg installation directory
    if (vcpkg_base_dir := Path(os.environ.get("VCPKG_BASE_DIR"))).is_dir():
        include_dirs.append(str(vcpkg_base_dir.joinpath("include").absolute()))
        include_dirs.append("C:/Program Files/valhalla/include")
        # DLLs are in the bin folder
        library_dirs.append(str(vcpkg_base_dir.joinpath("bin").absolute()))
        library_dirs.append(str(vcpkg_base_dir.joinpath("lib").absolute()))
        library_dirs.append("C:/Program Files/valhalla/lib")

# determine libraries to link to
if platform.system() == "Windows":
    libraries.extend([
        "valhalla",
        "libprotobuf-lite",
        "libcurl",
        "zlib",
        "gdal",
        "libspatialite",
        "sqlite3",
        "lz4",
        "geos",
        "luajit",
        "Ws2_32",
        "ole32",
        "Shell32"
    ])
    extra_compile_args.extend(["-DNOMINMAX", "-DWIN32_LEAN_AND_MEAN", "-DNOGDI"])
else: 
    extra_link_args.extend([
        "-lvalhalla",
        "-lprotobuf-lite",
        "-lcurl",
        "-lz",
        "-lprime_server",
        "-lzmq",
        "-lzmq",
        "-lgdal",
        "-lspatialite",
        "-lsqlite3",
        "-llz4",
        "-lgeos_c",
        "-lluajit-5.1",
    ])

ext_modules = [
    Pybind11Extension(
        "_valhalla",
        [os.path.join("src", "bindings", "python", "valhalla", "_valhalla.cc")],
        cxx_std=17,
        include_pybind11=False,  # use submodule'd pybind11
        library_dirs=library_dirs,
        include_dirs=include_dirs,
        extra_link_args=extra_link_args,
        extra_compile_args=extra_compile_args,
        libraries=libraries,
    ),
]

# open README.md for PyPI
with open(os.path.join(THIS_DIR, "README.md"), encoding="utf-8") as f:
    long_description = "\n" + f.read()

# if we push master, we upload to pyvalhalla-git
# this is used in GHA for publishing
pkg = os.environ.get('VALHALLA_RELEASE_PKG')
if not pkg or (pkg not in ["pyvalhalla", "pyvalhalla-git"]):
    warnings.warn(f"VALHALLA_RELEASE_PKG not set to a supported value: '{pkg}'")
    pkg = "pyvalhalla"

warnings.warn(f"Building package for {pkg} with $VALHALLA_RELEASE_PKG={os.environ.get('VALHALLA_RELEASE_PKG')}")

setup(
    name=pkg,
    description="High-level bindings to the Valhalla C++ library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Nils Nolde",
    author_email="nilsnolde+pyvalhalla@proton.me",
    packages=["valhalla"],
    package_dir={"": "./src/bindings/python"},
    python_requires=">=3.9.0",
    url="https://github.com/valhalla/valhalla",
    ext_modules=ext_modules,
    classifiers=[
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: Implementation :: CPython",
    ],
)
