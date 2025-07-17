import os
from pathlib import Path
import platform
import shutil
import sys
from typing import Optional

from pybind11.setup_helpers import Pybind11Extension
from setuptools import setup
# InWheel is platform-agnostic
from auditwheel.wheeltools import InWheel
from wheel.bdist_wheel import bdist_wheel as BDistWheelCommand  # noqa: E402

DEFAULT_VALHALLA_BUILD_DIR = "./build_manylinux"

THIS_DIR = Path(__file__).parent.resolve()
BINARIES = [
    "valhalla_service", # this needed a custom-built "patchelf" for auditwheel to properly fix
    "valhalla_build_tiles",
    "valhalla_build_admins",
    "valhalla_add_predicted_traffic",
    "valhalla_add_landmarks",
    "valhalla_ingest_transit",
    "valhalla_convert_transit",
]
IS_OSX = platform.system().lower() == "darwin"
IS_WIN = platform.system().lower() == "windows"
IS_LINUX = platform.system().lower() == "linux"

# verify build dir; needs to be an absolute path for InWheel context manager further below
# $VALHALLA_BUILD_BIN_DIR is set by GHA
valhalla_build_dir: Optional[Path] = Path(os.environ.get("VALHALLA_BUILD_BIN_DIR", DEFAULT_VALHALLA_BUILD_DIR)).absolute()
if not valhalla_build_dir.is_dir():
    print(f"[WARNING] Couldn't find $VALHALLA_BUILD_BIN_DIR={valhalla_build_dir} (default './build_manylinux'), skipping Valhalla executables...")
    valhalla_build_dir = None

# if we push master, we upload to pyvalhalla-weekly
# this is set in GHA when publishing
pkg = os.environ.get('VALHALLA_RELEASE_PKG')
if not pkg or (pkg not in ["pyvalhalla", "pyvalhalla-weekly"]):
    print(f"[WARNING] VALHALLA_RELEASE_PKG not set to a supported value: '{pkg}', defaulting to 'pyvalhalla-weekly'")
    pkg = "pyvalhalla-weekly"
print(f"Building package for {pkg} with $VALHALLA_RELEASE_PKG={os.environ.get('VALHALLA_RELEASE_PKG')}")


class ValhallaBDistWheelCommand(BDistWheelCommand):
    "Subclass to patch the wheel in ./dist and add Valhalla binaries"
    def run(self) -> None:
        super().run()

        # in the context of this class, valhalla_build_dir can't be None
        if not valhalla_build_dir.is_dir():
            print(f"[WARNING] No valid valhalla build directory, skipping adding executables..")
            return
        
        # get the wheel path
        impl_tag, abi_tag, plat_tag = self.get_tag()
        whl_basename = f"{self.wheel_dist_name}-{impl_tag}-{abi_tag}-{plat_tag}"
        whl_dist_path = Path(self.dist_dir).joinpath(whl_basename + ".whl")
        if not whl_dist_path.exists():
            print(f"[FATAL] is not an existing wheel: {whl_dist_path}")
            sys.exit(1)

        print(f"[INFO] Patching wheel {whl_dist_path} to include Valhalla binaries from {valhalla_build_dir}.")

        # copy the valhalla binaries and make sure we rewrite the RECORD file
        # NOTE, InWheel context changes the root dir, so all relative paths will be relative
        # to its temp dir
        source_bin_paths = [valhalla_build_dir.joinpath(binary + (".exe" if IS_WIN else "")) for binary in BINARIES]
        with InWheel(in_wheel=whl_dist_path, out_wheel=whl_dist_path) as in_wheel_path:
            wheel_bin_dir = in_wheel_path.joinpath("valhalla", "bin")
            wheel_bin_dir.mkdir()
            for source_bin_path in source_bin_paths:
                if not source_bin_path.is_file():
                    print(f"[WARNING] {source_bin_path} does not exist, skipping...")
                    continue
                wheel_binary_path = wheel_bin_dir.joinpath(source_bin_path.name)
                shutil.copy(source_bin_path, wheel_binary_path)
                print(f"[INFO] Copied {source_bin_path.name} into wheel at {wheel_binary_path}")

            print(f"[INFO] Updating RECORD file of {whl_dist_path}")


include_dirs = [
    str(THIS_DIR.joinpath("third_party", "date", "include")),
    str(THIS_DIR.joinpath("third_party", "pybind11", "include")),
    str(THIS_DIR.joinpath("third_party", "rapidjson", "include")),
    str(THIS_DIR.joinpath("valhalla")),
    str(THIS_DIR),
]
# this doesn't seem to be enough, on Linux we need to set LD_LIBRARY_PATH as well
# or auditwheel will link to e.g. system libgeos
library_dirs = ["/usr/local/lib", "/usr/local/lib64"]
libraries = list()
extra_link_args = list()
extra_compile_args = [f"-DVALHALLA_PYTHON_PACKAGE={pkg}"]
if version_modifier := os.environ.get("VALHALLA_VERSION_MODIFIER"):
    print(f"[INFO] Building with version modifier: {version_modifier}")
    extra_compile_args.append(f"-DVALHALLA_VERSION_MODIFIER={version_modifier}")

# determine the directories for compiling
if IS_OSX:
    include_dirs.append("/opt/homebrew/include")
    library_dirs.append("/opt/homebrew/lib")
elif IS_WIN:
    # this is set in GHA to the vcpkg installation directory
    if (vcpkg_base_dir := Path(os.environ.get("VCPKG_ARCH_ROOT", Path("/dev/null")))).is_dir():
        include_dirs.append(str(vcpkg_base_dir.joinpath("include").absolute()))
        include_dirs.append("C:/Program Files/valhalla/include")
        # DLLs are in the bin folder
        library_dirs.append(str(vcpkg_base_dir.joinpath("bin").absolute()))
        library_dirs.append(str(vcpkg_base_dir.joinpath("lib").absolute()))
        library_dirs.append("C:/Program Files/valhalla/lib")

# determine libraries to link to
# this is pretty annoying.. don't know any other way though..
if IS_WIN:
    libraries.extend([
        "valhalla",
        "libprotobuf-lite",
        "libcurl",
        "zlib",
        "gdal",
        "spatialite",
        "sqlite3",
        "lz4",
        "geos",
        "lua51",
        "Ws2_32",
        "ole32",
        "Shell32",
        "abseil_dll",
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

# config the C++ extension build
ext_modules = list()
for path, srcs in (("valhalla._valhalla", ("_valhalla",)), ("valhalla.utils.graph_utils", ("graph_utils", "graph_id"))):
    ext_modules.append(
        Pybind11Extension(
            path,
            list(map(lambda x: os.path.join("src", "bindings", "python", "src", x + ".cc"), srcs)),
            cxx_std=17,
            include_pybind11=False,  # use submodule'd pybind11
            library_dirs=library_dirs,
            include_dirs=include_dirs,
            extra_link_args=extra_link_args,
            extra_compile_args=extra_compile_args,
            libraries=libraries,
        )
    )

# open README.md for PyPI
with open(os.path.join(THIS_DIR, "src", "bindings", "python", "README.md"), encoding="utf-8") as f:
    long_description = "\n" + f.read()

script_entrypoints = dict()
if valhalla_build_dir is not None:
    # determine script entrypoints to wrap each Valhalla executable into an executable Python script,
    # e.g. 'valhalla_build_tiles -h' instead of 'python -m valhalla valhalla_build_tiles -h', i.e.
    # it'll install those Python wrapper scripts to the Python environment's ./bin folder
    # NOTE, if Valhalla executables are installed locally, those will usually be preferred
    # _scripts:run determines itself which executable is being run
    script_entrypoints = {
            'console_scripts': [f"{binary} = valhalla._scripts:run" for binary in BINARIES]
        }

setup(
    name=pkg,
    python_requires=">=3.9.0",
    description="High-level bindings to the Valhalla C++ library",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Nils Nolde",
    author_email="nilsnolde+pyvalhalla@proton.me",
    url="https://github.com/valhalla/valhalla",
    packages=["valhalla", "valhalla.utils"],
    package_dir={
        "": "src/bindings/python",
    },
    ext_modules=ext_modules,
    entry_points=script_entrypoints,
    # if we found executables we'll package them to let them go through auditing package (auditwheel etc.)
    cmdclass={"bdist_wheel": ValhallaBDistWheelCommand if valhalla_build_dir else BDistWheelCommand},
    classifiers=[
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: Implementation :: CPython",
    ],
)
