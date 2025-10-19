#!/usr/bin/env bash

set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 /path/to/addon.node /path/to/output_dir [--strip] [--tar]"
  exit 1
fi

BINDING_SRC="$(readlink -f "$1")"
OUT_DIR="$(readlink -m "$2")"
DO_STRIP="no"
MAKE_TAR="no"

shift 2 || true
while (( "$#" )); do
  case "$1" in
    --strip) DO_STRIP="yes" ;;
    --tar)   MAKE_TAR="yes" ;;
    *) echo "Unknown option: $1" ; exit 1 ;;
  esac
  shift
done

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Error: '$1' is required but not found in PATH." >&2
    exit 1
  }
}

require_cmd ldd
require_cmd patchelf
if [[ "$DO_STRIP" == "yes" ]]; then require_cmd strip; fi

if [[ ! -f "$BINDING_SRC" ]]; then
  echo "Error: binding not found: $BINDING_SRC" >&2
  exit 1
fi

file "$BINDING_SRC" | grep -q 'ELF' || {
  echo "Error: input is not an ELF shared object (.node): $BINDING_SRC" >&2
  exit 1
}

mkdir -p "$OUT_DIR/lib"

BINDING_DST="$OUT_DIR/$(basename "$BINDING_SRC")"
cp -f "$BINDING_SRC" "$BINDING_DST"

# Libraries we should NOT bundle (system/glibc pieces). Extend if needed.
# We'll skip anything matching these regexes (case-insensitive).
EXCLUDE_RE='^((linux-vdso\.so)|(ld-linux.*\.so)|(ld-musl.*\.so)|(libc\.so)|(libm\.so)|(libpthread\.so)|(librt\.so)|(libdl\.so)|(libnsl\.so)|(libresolv\.so)|(libutil\.so)|(libcrypt\.so)|(libanl\.so)|(libnss_.*\.so))$'

# Collect dependencies with ldd, return absolute paths (one per line)
collect_deps() {
  local elf="$1"
  # ldd output examples:
  #   libfoo.so.1 => /usr/lib/libfoo.so.1 (0x00007f...)
  #   libbar.so.2 (0x00007f...)                  # sometimes no path (not found or in vdso/loader)
  #   /lib64/ld-linux-x86-64.so.2 (0x00007f...)
  ldd "$elf" | awk '
    $2 == "=>" && $3 ~ /^\// { print $3 }
    $1 ~ /^\// { print $1 }
  ' | sort -u
}

# Copy a dependency into OUT_DIR/lib if allowed and not already present
copy_dep() {
  local dep_path="$1"
  local base="$(basename "$dep_path")"
  local base_lc="$(echo "$base" | tr '[:upper:]' '[:lower:]')"

  if [[ "$base_lc" =~ $EXCLUDE_RE ]]; then
    # Skip glibc/system loader bits
    return 0
  fi

  # If already copied, skip
  if [[ -f "$OUT_DIR/lib/$base" ]]; then
    return 0
  fi

  # Copy the actual file contents (follow symlinks to copy the real .so)
  cp -L "$dep_path" "$OUT_DIR/lib/$base"

  # If it was a symlink, also try to preserve the link name if useful
  if [[ -L "$dep_path" ]]; then
    # replicate symlink name pointing to the copied file (best-effort)
    local target="$(readlink "$dep_path")"
    if [[ -n "$target" && "$target" != "$base" ]]; then
      # Only create a link if target name differs and target exists in the lib dir
      if [[ -f "$OUT_DIR/lib/$(basename "$target")" ]]; then
        (cd "$OUT_DIR/lib" && ln -sf "$(basename "$target")" "$base") || true
      fi
    fi
  fi
}

# Recursively collect and copy deps of the binding and of any copied libs
# until a fixed point.
bundle_all_deps() {
  local queue=("$BINDING_DST")
  local seen=()

  while ((${#queue[@]})); do
    local cur="${queue[0]}"
    queue=("${queue[@]:1}")

    # Avoid re-processing
    if printf '%s\n' "${seen[@]}" | grep -qx "$cur"; then
      continue
    fi
    seen+=("$cur")

    # Get deps of current ELF
    while IFS= read -r dep; do
      [[ -f "$dep" ]] || continue
      local base="$(basename "$dep")"
      local base_lc="$(echo "$base" | tr '[:upper:]' '[:lower:]')"

      if [[ "$base_lc" =~ $EXCLUDE_RE ]]; then
        continue
      fi

      copy_dep "$dep"

      # Queue the copied dep for its own deps
      if [[ -f "$OUT_DIR/lib/$base" ]]; then
        queue+=("$OUT_DIR/lib/$base")
      fi
    done < <(collect_deps "$cur")
  done
}

echo "[1/4] Bundling dependencies..."
bundle_all_deps

echo "[2/4] Patching RPATHs..."
# Patch the binding to look into ./lib
patchelf --force-rpath --set-rpath '$ORIGIN/lib' "$BINDING_DST"

# Patch each bundled library to look within the lib dir for their own deps
shopt -s nullglob
for so in "$OUT_DIR"/lib/*.so*; do
  # Some .so files could be non-ELF (rare), skip with a quick check
  if file "$so" | grep -q 'ELF'; then
    patchelf --force-rpath --set-rpath '$ORIGIN' "$so" || {
      echo "Warning: failed to patchelf $so (continuing)"
    }
  fi
done
shopt -u nullglob

if [[ "$DO_STRIP" == "yes" ]]; then
  echo "[3/4] Stripping symbols (optional)..."
  strip --strip-unneeded "$BINDING_DST" || true
  for so in "$OUT_DIR"/lib/*.so*; do
    strip --strip-unneeded "$so" || true
  done
else
  echo "[3/4] Skipping strip (use --strip to enable)."
fi

if [[ "$MAKE_TAR" == "yes" ]]; then
  echo "[4/4] Creating tarball..."
  TAR_NAME="$(basename "$BINDING_DST" .node)-bundle.tar.gz"
  (cd "$OUT_DIR/.." && tar czf "$TAR_NAME" "$(basename "$OUT_DIR")")
  echo "Tarball: $(readlink -f "$OUT_DIR/../$TAR_NAME")"
else
  echo "[4/4] Done. Relocatable bundle at: $OUT_DIR"
fi

echo
echo "Summary:"
echo "  Binding: $BINDING_DST"
echo "  Lib dir: $OUT_DIR/lib"
echo "  RPATH(binding): \$ORIGIN/lib"
echo "  RPATH(libs):    \$ORIGIN"