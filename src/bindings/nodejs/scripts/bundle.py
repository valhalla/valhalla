#!/usr/bin/env python3

"""
This tool bundles native binaries with all their shared library dependencies for distribution.
It recursively discovers all dynamic libraries required by the input binaries (using ldd on Linux or
MachO analysis on macOS), copies them to a lib/ subdirectory, and patches RPATHs/install names so
they reference each other using relative paths. Finally, it strips debug symbols from all binaries
to minimize the bundle size, creating a self-contained package that can run on systems without
the original dependencies installed.
"""

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Set, List, Optional
from functools import cache
from macholib.MachO import MachO
from macholib.mach_o import LC_RPATH, LC_LOAD_DYLIB, LC_LOAD_WEAK_DYLIB, LC_REEXPORT_DYLIB

# Libraries we should NOT bundle (system/glibc pieces)
EXCLUDE_RE_LINUX = re.compile(
    r'^((linux-vdso\.so.*)|(ld-linux.*\.so.*)|(ld-musl.*\.so.*)|(libc\.so.*)|'
    r'(libm\.so.*)|(libpthread\.so.*)|(librt\.so.*)|(libdl\.so.*)|'
    r'(libnsl\.so.*)|(libresolv\.so.*)|(libutil\.so.*)|(libcrypt\.so.*)|'
    r'(libanl\.so.*)|(libnss_.*\.so.*)|(libgcc_s\.so.*)|(libstdc\+\+\.so.*)|'
    r'(libgomp\.so.*)|(libselinux\.so.*)|(libbsd\.so.*)|(libmd\.so.*))$',
    re.IGNORECASE
)


def is_macos() -> bool:
    return platform.system() == "Darwin"


def should_exclude_linux(dep: str) -> bool:
    basename = os.path.basename(dep)
    return EXCLUDE_RE_LINUX.match(basename.lower()) is not None

def should_exclude_macos(dep: str) -> bool:
    return dep.startswith("/System") or dep.startswith("/usr/lib")


def collect_deps_linux(elf_path: Path) -> List[str]:
    result = subprocess.run(
        ["ldd", str(elf_path)],
        capture_output=True,
        text=True,
        check=True
    )

    deps = []
    for line in result.stdout.splitlines():
        parts = line.strip().split()
        if len(parts) >= 3 and parts[1] == "=>" and parts[2].startswith("/"):
            deps.append(parts[2])
        elif len(parts) >= 1 and parts[0].startswith("/"):
            deps.append(parts[0])
    
    return sorted(set(dep for dep in deps if not should_exclude_linux(dep)))


def get_rpaths_macos(macho_path: Path) -> List[str]:
    macho = MachO(str(macho_path))
    rpaths = []
    
    for header in macho.headers:
        for cmd in header.commands:
            if cmd[0].cmd == LC_RPATH:
                # cmd[2] is the rpath string
                rpath = cmd[2].decode('utf-8').rstrip('\x00')
                rpaths.append(rpath)
    
    return rpaths


def resolve_rpath_macos(dep_name: str, macho_path: Path, rpaths: List[str]) -> Optional[str]:
    if not dep_name.startswith("@rpath/"):
        return dep_name

    # Extract the library name
    lib_name = dep_name[len("@rpath/"):]

    # Try to resolve using the binary's rpaths
    for rpath in rpaths:
        # Handle @loader_path and @executable_path in rpath
        rpath_resolved = rpath.replace("@loader_path", str(macho_path.parent))
        rpath_resolved = rpath_resolved.replace("@executable_path", str(macho_path.parent))

        candidate = os.path.join(rpath_resolved, lib_name)
        if os.path.exists(candidate):  
            return candidate 
    
    print(f"Warning: Could not resolve @rpath dependency: {dep_name} for {macho_path}", file=sys.stderr)
    return None


def resolve_loader_or_executable_path(dep_path: str, macho_path: Path) -> Optional[str]:
    for prefix in ["@loader_path/", "@executable_path/"]:
        if dep_path.startswith(prefix):
            lib_name = dep_path[len(prefix):]
            resolved = macho_path.parent / lib_name
            if resolved.is_file():
                return str(resolved)
            print(f"Warning: Could not resolve {prefix[:-1]} dependency: {dep_path} for {macho_path}", file=sys.stderr)
            return None
    return None

@cache
def collect_deps_macos_unresolved(macho_path: Path) -> List[str]:
    macho = MachO(str(macho_path))
    deps = []
    for header in macho.headers:
        for cmd in header.commands:
            # Check for various load commands that indicate dependencies
            if cmd[0].cmd in (LC_LOAD_DYLIB, LC_LOAD_WEAK_DYLIB, LC_REEXPORT_DYLIB):
                # cmd[2] is the library path string
                dep_path = cmd[2].decode('utf-8').rstrip('\x00')
                deps.append(dep_path)

    return sorted(set(dep for dep in deps if not should_exclude_macos(dep)))

def collect_deps_macos(macho_path: Path) -> List[str]:
    unresolved_deps = collect_deps_macos_unresolved(macho_path)
    rpaths = get_rpaths_macos(macho_path)
    deps = []
    
    for dep_path in unresolved_deps:
        if dep_path.startswith("/"):
            deps.append(dep_path)
        elif dep_path.startswith("@rpath/"):
            resolved = resolve_rpath_macos(dep_path, macho_path, rpaths)
            if resolved:
                deps.append(resolved)
        elif dep_path.startswith("@loader_path/") or dep_path.startswith("@executable_path/"):
            resolved = resolve_loader_or_executable_path(dep_path, macho_path)
            if resolved:
                deps.append(resolved)

    return sorted(set(deps))


def collect_deps(path: Path) -> List[str]:
    if is_macos():
        return collect_deps_macos(path)
    else:
        return collect_deps_linux(path)

def copy_dep(dep_path: str, out_dir: Path) -> None:
    lib_dir = out_dir / "lib"
    basename = os.path.basename(dep_path)
    
    dest_path = lib_dir / basename
    if dest_path.exists():
        return
    
    shutil.copy2(dep_path, dest_path, follow_symlinks=True)


def collect_deps_recursively(binary_path: Path) -> List[str]:
    all_deps: Set[str] = set()
    queue = [binary_path]
    
    while queue:
        cur = queue.pop(0)
        
        deps = collect_deps(cur)
        for dep in deps:
            if os.path.isfile(dep) and dep not in all_deps:
                all_deps.add(dep)
                queue.append(Path(dep))
    
    return list(all_deps)


def bundle_all_deps(binding_dst: Path, out_dir: Path) -> None:
    all_deps = collect_deps_recursively(binding_dst)

    for dep in all_deps:
        copy_dep(dep, out_dir)


def patch_rpath_linux_file(file_path: Path, rpath: str) -> None:
    subprocess.run(
        ["patchelf", "--force-rpath", "--set-rpath", rpath, str(file_path)],
        check=True
    )


def patch_rpaths_linux_libs(out_dir: Path) -> None:
    lib_dir = out_dir / "lib"
    for so in lib_dir.glob("*.so*"):
        patch_rpath_linux_file(so, "$ORIGIN")


def install_name_tool(operation: str, *args: str) -> None:
    subprocess.run(
        ["install_name_tool", operation] + list(args),
        capture_output=True,
        check=True
    )


def patch_rpath_macos_file(file_path: Path, out_dir: Path, is_in_lib_dir: bool = False) -> None:
    lib_dir = out_dir / "lib"
    
    # Get raw dependency paths without resolution (file has been moved)
    deps = collect_deps_macos_unresolved(file_path)
    for dep in deps:
        basename = os.path.basename(dep)
        if is_in_lib_dir:
            # Library referencing another library in the same dir
            install_name_tool("-change", dep, f"@loader_path/{basename}", str(file_path))
        else:
            # Binary/executable referencing a library in lib/
            install_name_tool("-change", dep, f"@loader_path/lib/{basename}", str(file_path))


def patch_rpaths_macos_libs(out_dir: Path) -> None:
    lib_dir = out_dir / "lib"
    
    # First, update the library IDs of all bundled libraries
    for dylib in lib_dir.glob("*.dylib"):
        install_name_tool("-id", f"@loader_path/{dylib.name}", str(dylib))

    # Update references in each library
    for dylib in lib_dir.glob("*.dylib"):
        patch_rpath_macos_file(dylib, out_dir, is_in_lib_dir=True)


def patch_rpaths(dst_files: List[Path], out_dir: Path) -> None:
    if is_macos():
        for dst_file in dst_files:
            patch_rpath_macos_file(dst_file, out_dir, is_in_lib_dir=False)
        
        patch_rpaths_macos_libs(out_dir)
    else:
        for dst_file in dst_files:
            patch_rpath_linux_file(dst_file, "$ORIGIN/lib")
        
        patch_rpaths_linux_libs(out_dir)


def strip_file(file_path: Path, strip_args: List[str]):
    # make writable
    os.chmod(file_path, os.stat(file_path).st_mode | 0o200)
    subprocess.run(
        ["strip"] + strip_args + [str(file_path)],
        capture_output=True,
        check=False
    )


def strip_symbols(binary_files: List[Path], out_dir: Path) -> None:
    strip_args = ["-x"] if is_macos() else ["--strip-unneeded"]
    
    # Strip all binary files
    for binary_file in binary_files:
        strip_file(binary_file, strip_args)
    
    # Strip libraries
    lib_dir = out_dir / "lib"
    lib_pattern = "*.dylib" if is_macos() else "*.so*"
    for lib in lib_dir.glob(lib_pattern):
        strip_file(lib, strip_args)


def codesign_file(file_path: Path) -> None:
    subprocess.run(
        ["codesign", "--sign", "-", "--force", str(file_path)],
        capture_output=True,
        check=True
    )

def codesign_binaries(binary_files: List[Path], out_dir: Path) -> None:
    if not is_macos():
        return
    
    # Sign all binary files
    for binary_file in binary_files:
        codesign_file(binary_file)
    
    # Sign all libraries
    lib_dir = out_dir / "lib"
    for lib in lib_dir.glob("*.dylib"):
        codesign_file(lib)




def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+")
    parser.add_argument("out_dir", help="Path to the output directory")
    
    args = parser.parse_args()

    input_files = [Path(file) for file in args.files]
    
    # Setup output directory
    out_dir = Path(args.out_dir)
    lib_dir = out_dir / "lib"
    lib_dir.mkdir(parents=True, exist_ok=True)

    
    # Copy all input files to output directory
    dst_files = []
    for src_file in input_files:
        dst_file = out_dir / src_file.name
        shutil.copy2(src_file, dst_file)
        dst_files.append(dst_file)
    
    print("Bundling dependencies from all files...")
    for dst_file in dst_files:
        bundle_all_deps(dst_file, out_dir)
    
    print("Patching RPATHs...")
    patch_rpaths(dst_files, out_dir)
    
    print("Stripping symbols...")
    strip_symbols(dst_files, out_dir)
    
    print("Code signing binaries...")
    codesign_binaries(dst_files, out_dir)



if __name__ == "__main__":
    main()

