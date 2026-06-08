# pyvalhalla — Python Bindings Guide

nanobind-based Python bindings for libvalhalla, shipped as the `pyvalhalla` wheel. **One compiled extension module** (`_valhalla`) + a thin pure-Python wrapper layer + an auto-generated `.pyi` type stub.

## Module Map

A single `NB_MODULE(_valhalla)` lives in [src/_valhalla.cc](src/_valhalla.cc) — a thin entry point that sets the version attr and calls one `init_x(module_&)` per binding into the same module. One `.so` means one nanobind runtime and one type registry, so bound types (`GraphId`, …) interop across all bindings. The `src/` tree mirrors the C++ modules; each binding dir (including `src/` root) owns a `module.h` declaring only its own `init_x` (ownership flows up — the entry point includes children, never the reverse). Include guards are path-derived (`PYVALHALLA_BALDR_UTILS_H`), not filename-derived, since every dir's header is named `module.h`.

| Symbol group | `init_x` (defining `.cc`) | Header | Wraps |
|---|---|---|---|
| exception | `pyvalhalla::init_exceptions` ([src/exceptions.cc](src/exceptions.cc)) | [src/module.h](src/module.h) | `ValhallaError` (manual `PyErr_NewExceptionWithDoc` + translator) |
| actor | `pyvalhalla::init_actor` ([src/actor.cc](src/actor.cc)) | [src/module.h](src/module.h) | `tyr::actor_t` → `_Actor` |
| graph id | `pyvalhalla::baldr::init_graphid` ([src/baldr/graph_id.cc](src/baldr/graph_id.cc)) | [src/baldr/module.h](src/baldr/module.h) | `baldr::GraphId` |
| graph reader | `pyvalhalla::baldr::utils::init_graphreader` ([src/baldr/utils/graph_reader.cc](src/baldr/utils/graph_reader.cc)) | [src/baldr/utils/module.h](src/baldr/utils/module.h) | `baldr::GraphReader` → `_GraphUtils` |
| graph tile | `pyvalhalla::baldr::utils::init_graphtile` ([src/baldr/utils/graph_tile.cc](src/baldr/utils/graph_tile.cc)) | [src/baldr/utils/module.h](src/baldr/utils/module.h) | `get_tile_*` helpers (`TileHierarchy`) |
| predicted speeds | `pyvalhalla::baldr::utils::init_predicted_speeds` ([src/baldr/utils/predicted_speeds.cc](src/baldr/utils/predicted_speeds.cc)) | [src/baldr/utils/module.h](src/baldr/utils/module.h) | `baldr::compress_speed_buckets`, DCT-II helpers |

All symbols land flat in `_valhalla`; the `valhalla/...` Python packages **mirror the C++ module tree** and provide the user-facing namespacing by re-exporting from `_valhalla` (e.g. `pyvalhalla::baldr::GraphId` → `valhalla.baldr.GraphId`; `pyvalhalla::baldr::utils::*` → `valhalla.baldr.utils.*`). Empty `src/{midgard,mjolnir,loki}/utils/` dirs are placeholders for future bindings.

Shared headers under [../shared/](../shared/) (e.g., `tile_id_utils.h`) are reused by the Node.js bindings.

| Python wrapper | Purpose |
|---|---|
| [valhalla/__init__.py](valhalla/__init__.py) | Re-exports `Actor`, `ValhallaError`, `get_config`, `get_help`, `__version__` |
| [valhalla/actor.py](valhalla/actor.py) | `Actor(_Actor)` — adds dict/str input handling via `@dict_or_str`, config validation. **Holds the user-facing docstrings** (see "Docstring placement" below) |
| [valhalla/config.py](valhalla/config.py) | `get_config`, `get_help`, `parse_and_validate_config` |
| [valhalla/baldr/__init__.py](valhalla/baldr/__init__.py) | Re-exports `GraphId` from `.._valhalla` |
| [valhalla/baldr/utils/__init__.py](valhalla/baldr/utils/__init__.py) | Public surface: re-exports tile helpers and DCT helpers straight from `..._valhalla`, plus `GraphUtils` from `.graph_utils` |
| [valhalla/baldr/utils/graph_utils.py](valhalla/baldr/utils/graph_utils.py) | `GraphUtils(_GraphUtils)` wrapper only (dict/Path/str config). No raw re-exports — those live in `__init__.py` |
| [valhalla/utils/__init__.py](valhalla/utils/__init__.py) | **Deprecation shim** — re-exports the relocated symbols from `valhalla.baldr[.utils]` and emits a `DeprecationWarning` on import. Remove in a future release. `decode_polyline` not covered (dropped) |
| [valhalla/__main__.py](valhalla/__main__.py), [valhalla/_scripts.py](valhalla/_scripts.py) | CLI entry points (installed only in scikit-build-core wheel builds — see [CMakeLists.txt](CMakeLists.txt) `SKBUILD` branch) |

## .pyi Stubs

`nanobind_add_stub` in [CMakeLists.txt](CMakeLists.txt) generates a single `_valhalla.pyi` directly into the source tree. It is **committed** and **refreshed on every build** — single source of truth is the compiled module (which in turn reads docstrings from the `.def(...)` calls).

`INCLUDE_PRIVATE` is set so the `_Actor` / `_GraphUtils` classes (underscore-prefixed) survive stubgen's default filter. The Python wrappers (`Actor`, `GraphUtils`) inherit from these and need them visible in the stub.

Install is straightforward: `install(FILES ${SRC}/.../X.pyi DESTINATION ...)` — pulls directly from the source tree, no build-dir intermediate. Ruff is told to skip `.pyi` files in both [`scripts/format.sh`](../../../scripts/format.sh) (`! -name "*.pyi"` in the find pattern) and [`pyproject.toml`](../../../pyproject.toml) (`extend-exclude`).

### Keeping stubs in sync

[`scripts/regenerate_python_stubs.sh`](../../../scripts/regenerate_python_stubs.sh):
1. Rebuilds the `_valhalla_stub` target in `BUILD_DIR` (default `build`, overridable by env var or first positional arg).
2. `git status --porcelain`s the `_valhalla.pyi` path; exits 1 with the diff if anything changed.

**CI** runs it in the `arm_build` job ([.github/workflows/linux.yml](../../../.github/workflows/linux.yml)) right after `make`. The build already produces the stubs as part of `ALL`, so the script's `cmake --build` is a no-op rebuild; the `git status` check is what fails the job. There is no local pre-commit hook — when you edit a binding source locally, rebuild before committing or CI will catch the drift.

## Docstring Placement (Important — Non-Obvious)

Pyright/Pylance reads docstrings **statically** and does **not** walk the MRO. So docstrings on a C++-bound parent class (`_Actor`) do not surface on hover for the Python subclass (`Actor`) in VSCode, even though `help(Actor)` works at runtime.

The convention:
- **User-facing classes and methods** that have a Python wrapper → docstring as a string literal in the Python source (e.g., [valhalla/actor.py](valhalla/actor.py)). Pyright sees the literal, VSCode hover works.
- **Bindings without a Python wrapper** (currently: `_Actor.optimized_route`, exception classes, the `get_tile_*` and DCT free functions) → docstring stays in the C++ `.def(...)` / `nb::class_(...)` arg. The generated `.pyi` carries it.

Don't add C++ docstrings to methods that also have a Python wrapper with its own docstring — that duplicates and drifts.

## Add a new binding function

1. Add `.def(...)` to the relevant `.cc` (or a new `m.def(...)` for free functions).
2. If it gets a Python wrapper (e.g., `dict_or_str` handling), add it to the matching Python class and put the docstring there. Otherwise put the docstring in the `.def(...)`.
3. Rebuild — the `*_stub` target regenerates the `.pyi` automatically.
4. Commit both the `.cc` and the `.pyi`. CI will fail the PR if you forget the stub.

## Add a new binding group (init_x)

There is one extension module. To add bindings for another C++ area, add an `init_x` and wire it in — do **not** add a second `nanobind_add_module`.

1. Create `src/<module>/[utils/]<name>.cc` defining `void init_<x>(nanobind::module_& m)` in namespace `pyvalhalla::<module>[::utils]`. Declare it in that dir's `module.h` (create one if the dir doesn't have it yet, with a path-derived include guard) — the header declares only symbols defined in its own directory.
2. In [src/_valhalla.cc](src/_valhalla.cc): `#include` the dir's `module.h` (e.g. `"midgard/module.h"`) and call `init_<x>(m)` at the end of `NB_MODULE`.
3. Add the new `.cc` to the `_valhalla` source list in [CMakeLists.txt](CMakeLists.txt). No new target, stub, or install needed — it's all one module.
4. Rebuild — the single `_valhalla_stub` regenerates `_valhalla.pyi`. Commit both the `.cc` and the `.pyi`.
5. If it has Python-facing classes, write a wrapper under `valhalla/...` and re-export from the right `__init__.py` (import the bound symbols from `.._valhalla`).

## Build & Install for Local Dev

```bash
# Configure once (sets up build dir)
cmake -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_PYTHON_BINDINGS=On

# Install build deps into your venv (pyproject.toml's [build-system].requires)
pip install -r src/bindings/python/requirements-build.txt

# Editable install — keeps reusing build/, fast iteration
pip install -e . --no-build-isolation \
  -Cbuild-dir=build \
  -Ccmake.build-type=Release \
  -Ccmake.define.VALHALLA_VERSION_MODIFIER="$(git rev-parse --short HEAD)"
```

`--no-build-isolation` means pip uses the active venv as-is, so the build-deps must already be installed there. Without it, pip creates a temp env per invocation and your `build-dir` is harder to reuse.

Note: `valhalla` (the main C++ library) is built `POSITION_INDEPENDENT_CODE ON` when the python bindings are enabled, so the extension can statically link against the right `.o`s. Even with `BUILD_SHARED_LIBS=Off` on the main library, the module uses it via PIC.

## Build / ABI Details

- **`STABLE_ABI`** on `_valhalla` → abi3 wheels, one wheel per Python major. `wheel.py-api = "cp312"` in [pyproject.toml](../../../pyproject.toml).
- **`NB_STATIC`** → nanobind's runtime is statically linked into the module (no separate `libnanobind.so`).
- **`LTO`** is **not** enabled. `graph_id.cc` (now part of `_valhalla`) pulls in heavy `baldr/graphreader.h` templates that trigger a GCC 14 ARM64 LTO ICE (`internal compiler error: in get_token`); since LTO is a per-module link-time setting, that constraint applies to the whole extension. See the comment in [CMakeLists.txt](CMakeLists.txt) above the `_valhalla` target.
- **`RPATH`** set to `$ORIGIN` (Linux) / `@loader_path` (macOS) so the `.abi3.so` finds the sibling `libvalhalla.so` inside the wheel.
- **`LIBRARY_OUTPUT_DIRECTORY`** places the module under `${BIN}/valhalla/` so the build tree mirrors the install layout.

## Tests

Located under [../../../test/bindings/python/](../../../test/bindings/python/):

| File | Notes |
|---|---|
| `test_actor.py` | Needs `test/data/utrecht_tiles/` — pre-built tileset. All-or-nothing: `setUpClass` builds an Actor, so without tiles every test errors. |
| `test_graph_utils.py` | Mostly tile-independent; `test_get_edge_shape` + the `graphutils_*` tests need utrecht_tiles. |
| `test_predicted_speeds.py` | Self-contained, no tiles needed. |
| `test_config.py` | Self-contained. |

Run with `PYTHONPATH=build/.../src/bindings/python python -m pytest ...`. Watch out for an already-installed `valhalla` in your venv shadowing the PYTHONPATH — the venv's `valhalla/__init__.py` wins because it's a proper package, not just on sys.path.

Pre-existing segfault on `test_get_tile_ids_from_ring_*` reproduces on master — unrelated to recent work.

## Exceptions

`ValhallaError` is created manually with `PyErr_NewExceptionWithDoc` (not `nb::exception`) so the translator can populate structured attributes (`code`, `http_code`, `message`, `http_message`) on the instance. See the `nb::register_exception_translator` block in [src/exceptions.cc](src/exceptions.cc). Don't switch to `nb::exception` without preserving these fields — downstream users read them.

## Wheels

`pyproject.toml` drives cibuildwheel. Linux uses a custom manylinux image. The `auditwheel` patch in [scripts/auditwheel_patch.py](scripts/auditwheel_patch.py) allows vendoring libraries that are also linked to libpython (which auditwheel normally refuses). See the `[tool.cibuildwheel.linux]` block.
