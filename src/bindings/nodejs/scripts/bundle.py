#!/usr/bin/env python3

import argparse
import os
import platform
import re
import shutil
import subprocess
import sys
import tarfile
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
    """Check if a command exists in PATH."""
    if shutil.which(cmd) is None:
        print(f"Error: '{cmd}' is required but not found in PATH.", file=sys.stderr)
        sys.exit(1)


def is_macos() -> bool:
    """Check if running on macOS."""
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
    """Collect dependencies using otool on macOS."""
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
    """Collect dependencies based on platform."""
    if is_macos():
        return collect_deps_macos(path)
    else:
        return collect_deps_linux(path)


def should_exclude(dep: str) -> bool:
    """Check if a library should be excluded from bundling."""
    if is_macos():
        if dep.startswith("/System") or dep.startswith("/usr/lib"):
            return True
        return False
    else:
        basename = os.path.basename(dep)
        return EXCLUDE_RE_LINUX.match(basename.lower()) is not None


def copy_dep(dep_path: str, out_dir: Path) -> None:
    """Copy a dependency to OUT_DIR/lib if allowed and not already present."""
    lib_dir = out_dir / "lib"
    basename = os.path.basename(dep_path)
    
    if should_exclude(basename):
        return
    
    dest_path = lib_dir / basename
    if dest_path.exists():
        return
    
    # Copy the actual file contents (follow symlinks)
    shutil.copy2(dep_path, dest_path, follow_symlinks=True)
    
    # If it was a symlink, try to preserve the link name
    if os.path.islink(dep_path):
        target = os.readlink(dep_path)
        target_basename = os.path.basename(target)
        if target and target_basename != basename:
            target_path = lib_dir / target_basename
            if target_path.exists():
                link_path = lib_dir / basename
                if link_path.exists():
                    link_path.unlink()
                try:
                    link_path.symlink_to(target_basename)
                except Exception:
                    pass


def collect_deps_recursively(binary_path: str) -> List[str]:
    """Recursively collect all dependencies of a binary."""
    all_deps = []
    queue = [binary_path]
    seen: Set[str] = set()
    
    while queue:
        cur = queue.pop(0)
        
        if cur in seen:
            continue
        seen.add(cur)
        
        # Get deps of current binary
        deps = collect_deps(cur)
        for dep in deps:
            if not os.path.isfile(dep):
                continue
            
            if should_exclude(dep):
                continue
            
            # Add to result if not already there
            if dep not in all_deps:
                all_deps.append(dep)
            
            # Queue for recursive processing
            if dep not in seen:
                queue.append(dep)
    
    return all_deps


def bundle_all_deps(binding_dst: Path, out_dir: Path) -> None:
    """Recursively collect and copy dependencies."""
    # Collect all dependencies first

    all_deps = collect_deps_recursively(str(binding_dst))

    print('all_deps:', all_deps)
    for dep in all_deps:
        print('dep:', dep)

    # Copy all collected dependencies
    for dep in all_deps:
        copy_dep(dep, out_dir)


def patch_rpaths_linux(binding_dst: Path, out_dir: Path) -> None:
    """Patch RPATHs on Linux using patchelf."""
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


def codesign_adhoc(path: Path) -> None:
    """Re-sign a binary with ad-hoc signature to fix invalidated signatures."""
    try:
        subprocess.run(
            ["codesign", "--force", "--sign", "-", str(path)],
            capture_output=True,
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"Warning: failed to re-sign {path}: {e.stderr.decode()}")


def patch_rpaths_macos(binding_dst: Path, out_dir: Path) -> None:
    """Patch install names on macOS using install_name_tool."""
    lib_dir = out_dir / "lib"
    modified_files = set()
    
    # First, update the library IDs of all bundled libraries
    for dylib in lib_dir.glob("*.dylib"):
        try:
            subprocess.run(
                ["install_name_tool", "-id", f"@loader_path/{dylib.name}", str(dylib)],
                capture_output=True,
                check=True
            )
            modified_files.add(dylib)
        except subprocess.CalledProcessError:
            print(f"Warning: failed to set ID for {dylib} (continuing)")
    
    # Update references in the binding
    deps = collect_deps_macos(str(binding_dst))
    binding_modified = False
    for dep in deps:
        basename = os.path.basename(dep)
        if (lib_dir / basename).exists():
            try:
                subprocess.run(
                    ["install_name_tool", "-change", dep, f"@loader_path/lib/{basename}", str(binding_dst)],
                    capture_output=True,
                    check=True
                )
                binding_modified = True
            except subprocess.CalledProcessError:
                pass
    
    if binding_modified:
        modified_files.add(binding_dst)
    
    # Update references in each library
    for dylib in lib_dir.glob("*.dylib"):
        deps = collect_deps_macos(str(dylib))
        dylib_modified = False
        for dep in deps:
            basename = os.path.basename(dep)
            if (lib_dir / basename).exists():
                try:
                    subprocess.run(
                        ["install_name_tool", "-change", dep, f"@loader_path/{basename}", str(dylib)],
                        capture_output=True,
                        check=True
                    )
                    dylib_modified = True
                except subprocess.CalledProcessError:
                    pass
        
        if dylib_modified:
            modified_files.add(dylib)
    
    # Re-sign all modified files with ad-hoc signature
    for path in modified_files:
        codesign_adhoc(path)


def patch_rpaths(binding_dst: Path, out_dir: Path) -> None:
    """Patch RPATHs/install names based on platform."""
    if is_macos():
        patch_rpaths_macos(binding_dst, out_dir)
    else:
        patch_rpaths_linux(binding_dst, out_dir)


def strip_symbols(binding_dst: Path, out_dir: Path) -> None:
    """Strip symbols from binaries."""
    strip_cmd = "strip"
    strip_args = ["-x"] if is_macos() else ["--strip-unneeded"]
    stripped_files = []
    
    # Strip the binding
    try:
        # Make writable first
        os.chmod(binding_dst, 0o755)
        subprocess.run(
            [strip_cmd] + strip_args + [str(binding_dst)],
            capture_output=True,
            check=False
        )
        stripped_files.append(binding_dst)
    except Exception:
        pass
    
    # Strip libraries
    lib_dir = out_dir / "lib"
    for lib in lib_dir.glob("*.so*" if not is_macos() else "*.dylib"):
        try:
            # Make writable first
            os.chmod(lib, 0o755)
            subprocess.run(
                [strip_cmd] + strip_args + [str(lib)],
                capture_output=True,
                check=False
            )
            stripped_files.append(lib)
        except Exception:
            pass
    
    # Re-sign stripped files on macOS (strip also invalidates signatures)
    if is_macos():
        for path in stripped_files:
            codesign_adhoc(path)


def strip_symbols_multi(binary_files: List[Path], out_dir: Path) -> None:
    """Strip debug symbols from multiple binaries and libraries."""
    strip_cmd = "strip"
    strip_args = ["-x"] if is_macos() else ["--strip-unneeded"]
    stripped_files = []
    
    # Strip all binary files
    for binary_file in binary_files:
        try:
            os.chmod(binary_file, 0o755)
            subprocess.run(
                [strip_cmd] + strip_args + [str(binary_file)],
                capture_output=True,
                check=False
            )
            stripped_files.append(binary_file)
        except Exception:
            pass
    
    # Strip libraries
    lib_dir = out_dir / "lib"
    for lib in lib_dir.glob("*.so*" if not is_macos() else "*.dylib"):
        try:
            os.chmod(lib, 0o755)
            subprocess.run(
                [strip_cmd] + strip_args + [str(lib)],
                capture_output=True,
                check=False
            )
            stripped_files.append(lib)
        except Exception:
            pass
    
    # Re-sign stripped files on macOS (strip also invalidates signatures)
    if is_macos():
        for path in stripped_files:
            codesign_adhoc(path)


def create_tarball(out_dir: Path, binding_dst: Path) -> str:
    """Create a tarball of the bundle."""
    binding_name = binding_dst.stem  # without .node extension
    tar_name = f"{binding_name}-bundle.tar.gz"
    tar_path = out_dir.parent / tar_name
    
    with tarfile.open(tar_path, "w:gz") as tar:
        tar.add(out_dir, arcname=out_dir.name)
    
    return str(tar_path.resolve())


def create_tarball_multi(out_dir: Path, binary_files: List[Path]) -> str:
    """Create a tarball of the bundle with multiple files."""
    # Use first .node file or first binary for naming
    node_file = next((f for f in binary_files if f.name.endswith('.node')), binary_files[0])
    bundle_name = node_file.stem  # without extension
    tar_name = f"{bundle_name}-bundle.tar.gz"
    tar_path = out_dir.parent / tar_name
    
    with tarfile.open(tar_path, "w:gz") as tar:
        tar.add(out_dir, arcname=out_dir.name)
    
    return str(tar_path.resolve())


def main():
    parser = argparse.ArgumentParser(
        description="Bundle Node.js addon and executables with their dependencies"
    )
    parser.add_argument("files", nargs="+", help="Path to the files to bundle (addon.node and/or executables)")
    parser.add_argument("out_dir", help="Path to the output directory")
    parser.add_argument("--strip", action="store_true", help="Strip symbols from binaries")
    parser.add_argument("--tar", action="store_true", help="Create a tarball of the bundle")
    
    args = parser.parse_args()
    
    # Check required commands
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
    
    print(f"[INFO] Bundling {len(input_files)} file(s)...")
    for src_file in input_files:
        print(f"  - {src_file.name}")
    print()
    
    # Copy all files to output directory
    dst_files = []
    for src_file in input_files:
        dst_file = out_dir / src_file.name
        shutil.copy2(src_file, dst_file)
        # Make executable if it wasn't .node addon
        if not src_file.name.endswith('.node'):
            os.chmod(dst_file, 0o755)
        dst_files.append(dst_file)
    
    # Bundle dependencies from ALL files into shared lib/ directory
    print("[1/4] Bundling dependencies from all files...")
    for dst_file in dst_files:
        print(f"  Processing: {dst_file.name}")
        bundle_all_deps(dst_file, out_dir)
    
    # Patch RPATHs for all files
    print("[2/4] Patching RPATHs for all files...")
    for dst_file in dst_files:
        print(f"  Patching: {dst_file.name}")
        patch_rpaths(dst_file, out_dir)
    
    # Strip symbols
    if args.strip:
        print("[3/4] Stripping symbols (optional)...")
        strip_symbols_multi(dst_files, out_dir)
    else:
        print("[3/4] Skipping strip (use --strip to enable).")
    
    # Create tarball
    if args.tar:
        print("[4/4] Creating tarball...")
        tar_path = create_tarball_multi(out_dir, dst_files)
        print(f"Tarball: {tar_path}")
    else:
        print(f"[4/4] Done. Relocatable bundle at: {out_dir}")
    
    # Print summary
    print()
    print("Summary:")
    print(f"  Files bundled: {len(dst_files)}")
    for dst_file in dst_files:
        print(f"    - {dst_file.name}")
    print(f"  Lib dir: {lib_dir}")
    lib_count = len(list(lib_dir.glob("*.so*" if not is_macos() else "*.dylib")))
    print(f"  Shared libraries: {lib_count}")
    if is_macos():
        print("  Install names: @loader_path/lib (binaries)")
        print("  Install names: @loader_path (libs)")
    else:
        print("  RPATH(binaries): $ORIGIN/lib")
        print("  RPATH(libs):     $ORIGIN")


if __name__ == "__main__":
    main()

