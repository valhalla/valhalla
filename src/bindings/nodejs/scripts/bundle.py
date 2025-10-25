#!/usr/bin/env python3

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Set, List


# Libraries we should NOT bundle (system/glibc pieces)
EXCLUDE_RE_LINUX = re.compile(
    r'^((linux-vdso\.so.*)|(ld-linux.*\.so.*)|(ld-musl.*\.so.*)|(libc\.so.*)|'
    r'(libm\.so.*)|(libpthread\.so.*)|(librt\.so.*)|(libdl\.so.*)|'
    r'(libnsl\.so.*)|(libresolv\.so.*)|(libutil\.so.*)|(libcrypt\.so.*)|'
    r'(libanl\.so.*)|(libnss_.*\.so.*)|(libgcc_s\.so.*)|(libstdc\+\+\.so.*)|'
    r'(libgomp\.so.*)|(libselinux\.so.*)|(libbsd\.so.*)|(libmd\.so.*))$',
    re.IGNORECASE
)


def require_cmd(cmd: str) -> None:
    if shutil.which(cmd) is None:
        print(f"Error: '{cmd}' is required but not found in PATH.", file=sys.stderr)
        sys.exit(1)


def is_macos() -> bool:
    return platform.system() == "Darwin"


def collect_deps_linux(elf_path: str) -> List[str]:
    """Collect dependencies using ldd on Linux."""
    try:
        result = subprocess.run(
            ["ldd", elf_path],
            capture_output=True,
            text=True,
            check=True
        )
    except subprocess.CalledProcessError:
        return []
    
    deps = []
    for line in result.stdout.splitlines():
        parts = line.strip().split()
        if len(parts) >= 3 and parts[1] == "=>" and parts[2].startswith("/"):
            deps.append(parts[2])
        elif len(parts) >= 1 and parts[0].startswith("/"):
            deps.append(parts[0])
    
    return sorted(set(deps))


def get_rpaths_macos(macho_path: str) -> List[str]:
    """Get the rpath entries from a Mach-O binary."""
    try:
        result = subprocess.run(
            ["otool", "-l", macho_path],
            capture_output=True,
            text=True,
            check=True
        )
    except subprocess.CalledProcessError:
        return []

    rpaths = []
    lines = result.stdout.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line == "cmd LC_RPATH":
            # The path is in the next few lines, look for "path"
            for j in range(i + 1, min(i + 5, len(lines))):
                if lines[j].strip().startswith("path"):
                    # Format is usually "path /some/path (offset 12)"
                    parts = lines[j].strip().split(maxsplit=1)
                    if len(parts) > 1:
                        path = parts[1].split("(")[0].strip()
                        rpaths.append(path)
                    break
        i += 1

    return rpaths


def resolve_rpath_macos(dep_name: str, macho_path: str, rpaths: List[str]) -> str:
    """Resolve an @rpath dependency to an actual filesystem path."""
    if not dep_name.startswith("@rpath/"):
        return dep_name

    # Extract the library name
    lib_name = dep_name[7:]  # Remove "@rpath/"

    # Try to resolve using the binary's rpaths
    for rpath in rpaths:
        # Handle @loader_path and @executable_path in rpath
        rpath_resolved = rpath.replace("@loader_path", os.path.dirname(macho_path))
        rpath_resolved = rpath_resolved.replace("@executable_path", os.path.dirname(macho_path))

        candidate = os.path.join(rpath_resolved, lib_name)
        if os.path.isfile(candidate):
            return os.path.realpath(candidate)

    # Fallback: try common paths on macOS
    common_paths = [
        "/usr/local/lib",
        "/opt/homebrew/lib",
        "/usr/local/opt",
        os.path.dirname(macho_path)
    ]
    
    for base_path in common_paths:
        candidate = os.path.join(base_path, lib_name)
        if os.path.isfile(candidate):
            return os.path.realpath(candidate)
    
    return ""


def collect_deps_macos(macho_path: str) -> List[str]:
    try:
        result = subprocess.run(
            ["otool", "-L", macho_path],
            capture_output=True,
            text=True,
            check=True
        )
    except subprocess.CalledProcessError:
        return []

    # Get rpaths for resolving @rpath dependencies
    rpaths = get_rpaths_macos(macho_path)

    deps = []
    lines = result.stdout.splitlines()
    # Skip first line (it's the file path)
    for line in lines[1:]:
        parts = line.strip().split()
        if not parts:
            continue

        dep_path = parts[0]

        # Handle absolute paths
        if dep_path.startswith("/"):
            deps.append(dep_path)
        # Handle @rpath dependencies
        elif dep_path.startswith("@rpath/"):
            resolved = resolve_rpath_macos(dep_path, macho_path, rpaths)
            if resolved:
                deps.append(resolved)
            else:
                print(f"Warning: Could not resolve @rpath dependency: {dep_path} for {macho_path}", file=sys.stderr)
        # Handle @loader_path dependencies
        elif dep_path.startswith("@loader_path/"):
            lib_name = dep_path[13:]  # Remove "@loader_path/"
            resolved = os.path.join(os.path.dirname(macho_path), lib_name)
            if os.path.isfile(resolved):
                deps.append(os.path.realpath(resolved))
            else:
                print(f"Warning: Could not resolve @loader_path dependency: {dep_path} for {macho_path}", file=sys.stderr)
        # Handle @executable_path dependencies
        elif dep_path.startswith("@executable_path/"):
            lib_name = dep_path[17:]  # Remove "@executable_path/"
            resolved = os.path.join(os.path.dirname(macho_path), lib_name)
            if os.path.isfile(resolved):
                deps.append(os.path.realpath(resolved))
            else:
                print(f"Warning: Could not resolve @executable_path dependency: {dep_path} for {macho_path}", file=sys.stderr)

    return sorted(set(deps))


def collect_deps(path: str) -> List[str]:
    if is_macos():
        return collect_deps_macos(path)
    else:
        return collect_deps_linux(path)


def should_exclude(dep: str) -> bool:
    if is_macos():
        if dep.startswith("/System") or dep.startswith("/usr/lib"):
            return True
        return False
    else:
        basename = os.path.basename(dep)
        return EXCLUDE_RE_LINUX.match(basename.lower()) is not None


def copy_dep(dep_path: str, out_dir: Path) -> None:
    lib_dir = out_dir / "lib"
    basename = os.path.basename(dep_path)
    
    if should_exclude(basename):
        return
    
    dest_path = lib_dir / basename
    if dest_path.exists():
        return
    
    print(f"Copying dependency: {dep_path} to {dest_path}")
    shutil.copy2(dep_path, dest_path, follow_symlinks=True)


def collect_deps_recursively(binary_path: str) -> List[str]:
    all_deps: Set[str] = set()
    queue = [binary_path]
    
    while queue:
        cur = queue.pop(0)
        
        deps = collect_deps(cur)
        for dep in deps:
            if os.path.isfile(dep) and not should_exclude(dep) and dep not in all_deps:

                print(f"Adding dependency: {dep} from {cur}")
                all_deps.add(dep)
                queue.append(dep)
    
    return list(all_deps)


def bundle_all_deps(binding_dst: Path, out_dir: Path) -> None:
    all_deps = collect_deps_recursively(str(binding_dst))

    for dep in all_deps:
        copy_dep(dep, out_dir)


def patch_rpaths_linux(binding_dst: Path, out_dir: Path) -> None:
    # Patch the binding to look into ./lib
    subprocess.run(
        ["patchelf", "--force-rpath", "--set-rpath", "$ORIGIN/lib", str(binding_dst)],
        check=True
    )
    
    # Patch each bundled library to look within the lib dir
    lib_dir = out_dir / "lib"
    for so in lib_dir.glob("*.so*"):
        # Check if it's an ELF file
        try:
            result = subprocess.run(
                ["file", str(so)],
                capture_output=True,
                text=True,
                check=True
            )
            if "ELF" in result.stdout:
                try:
                    subprocess.run(
                        ["patchelf", "--force-rpath", "--set-rpath", "$ORIGIN", str(so)],
                        check=True
                    )
                except subprocess.CalledProcessError:
                    print(f"Warning: failed to patchelf {so} (continuing)")
        except subprocess.CalledProcessError:
            continue


def patch_rpaths_macos(binding_dst: Path, out_dir: Path) -> None:
    """Patch install names on macOS using install_name_tool."""
    lib_dir = out_dir / "lib"
    
    # First, update the library IDs of all bundled libraries
    for dylib in lib_dir.glob("*.dylib"):
        try:
            subprocess.run(
                ["install_name_tool", "-id", f"@loader_path/{dylib.name}", str(dylib)],
                capture_output=True,
                check=True
            )
        except subprocess.CalledProcessError:
            print(f"Warning: failed to set ID for {dylib} (continuing)")
    
    # Update references in the binding
    deps = collect_deps_macos(str(binding_dst))
    for dep in deps:
        basename = os.path.basename(dep)
        if (lib_dir / basename).exists():
            try:
                subprocess.run(
                    ["install_name_tool", "-change", dep, f"@loader_path/lib/{basename}", str(binding_dst)],
                    capture_output=True,
                    check=True
                )
            except subprocess.CalledProcessError:
                pass
    
    # Update references in each library
    for dylib in lib_dir.glob("*.dylib"):
        deps = collect_deps_macos(str(dylib))
        for dep in deps:
            basename = os.path.basename(dep)
            if (lib_dir / basename).exists():
                try:
                    subprocess.run(
                        ["install_name_tool", "-change", dep, f"@loader_path/{basename}", str(dylib)],
                        capture_output=True,
                        check=True
                    )
                except subprocess.CalledProcessError:
                    pass



def patch_rpaths(binding_dst: Path, out_dir: Path) -> None:
    """Patch RPATHs/install names based on platform."""
    if is_macos():
        patch_rpaths_macos(binding_dst, out_dir)
    else:
        patch_rpaths_linux(binding_dst, out_dir)


def strip_file(file_path: Path, strip_cmd: str, strip_args: List[str]) -> bool:
    try:
        os.chmod(file_path, 0o755)
        subprocess.run(
            [strip_cmd] + strip_args + [str(file_path)],
            capture_output=True,
            check=False
        )
        return True
    except Exception:
        return False


def strip_symbols(binary_files: List[Path], out_dir: Path) -> None:
    strip_cmd = "strip"
    strip_args = ["-x"] if is_macos() else ["--strip-unneeded"]
    stripped_files = []
    
    # Strip all binary files
    for binary_file in binary_files:
        if strip_file(binary_file, strip_cmd, strip_args):
            stripped_files.append(binary_file)
    
    # Strip libraries
    lib_dir = out_dir / "lib"
    lib_pattern = "*.dylib" if is_macos() else "*.so*"
    for lib in lib_dir.glob(lib_pattern):
        if strip_file(lib, strip_cmd, strip_args):
            stripped_files.append(lib)




def main():
    parser = argparse.ArgumentParser(
        description="Bundle Node.js addon and executables with their dependencies"
    )
    parser.add_argument("files", nargs="+", help="Path to the files to bundle (addon.node and/or executables)")
    parser.add_argument("out_dir", help="Path to the output directory")
    
    args = parser.parse_args()
    
    if is_macos():
        require_cmd("otool")
        require_cmd("install_name_tool")
    else:
        require_cmd("ldd")
        require_cmd("patchelf")
    
    # Validate input files
    input_files = []
    for file_path in args.files:
        src_path = Path(file_path)
        if not src_path.is_file():
            print(f"Error: file not found: {src_path}", file=sys.stderr)
            sys.exit(1)
        input_files.append(src_path)
    
    if not input_files:
        print("Error: no files specified", file=sys.stderr)
        sys.exit(1)
    
    # Setup output directory
    out_dir = Path(args.out_dir)
    lib_dir = out_dir / "lib"
    lib_dir.mkdir(parents=True, exist_ok=True)

    
    # Copy all files to output directory
    dst_files = []
    for src_file in input_files:
        dst_file = out_dir / src_file.name
        shutil.copy2(src_file, dst_file)
        # Make executable if it wasn't .node addon
        if not src_file.name.endswith('.node'):
            os.chmod(dst_file, 0o755)
        dst_files.append(dst_file)
    
    print("Bundling dependencies from all files...")
    for dst_file in dst_files:
        bundle_all_deps(dst_file, out_dir)
    
    print("Patching RPATHs for all files...")
    for dst_file in dst_files:
        patch_rpaths(dst_file, out_dir)
    
    print("Stripping symbols...")
    strip_symbols(dst_files, out_dir)



if __name__ == "__main__":
    main()

