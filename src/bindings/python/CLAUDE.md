# pyvalhalla — Python Bindings Guide

nanobind-based Python bindings for libvalhalla, shipped as the `pyvalhalla` wheel. Three compiled extension modules + a thin pure-Python wrapper layer + auto-generated `.pyi` type stubs.

## Module Map

| Compiled module | File system path | Sources | Wraps |
|---|---|---|---|
| `_valhalla` | `valhalla/_valhalla.abi3.so` | [src/_valhalla.cc](src/_valhalla.cc) | `tyr::actor_t` → `_Actor`, `ValhallaError` |
| `_graph_utils` | `valhalla/utils/_graph_utils.abi3.so` | [src/graph_utils.cc](src/graph_utils.cc) (`NB_MODULE`), [src/graph_id.cc](src/graph_id.cc) (`init_graphid`), [src/graph_utils_module.h](src/graph_utils_module.h) | `baldr::GraphId`, `baldr::GraphReader` → `_GraphUtils`, `get_tile_*` helpers |
| `predicted_speeds` | `valhalla/utils/predicted_speeds.abi3.so` | [src/predicted_speeds.cc](src/predicted_speeds.cc) | `baldr::compress_speed_buckets`, DCT-II helpers |

Shared headers under [../shared/](../shared/) (e.g., `tile_id_utils.h`) are reused by the Node.js bindings.

| Python wrapper | Purpose |
|---|---|
| [valhalla/__init__.py](valhalla/__init__.py) | Re-exports `Actor`, `ValhallaError`, `get_config`, `get_help`, `__version__` |
| [valhalla/actor.py](valhalla/actor.py) | `Actor(_Actor)` — adds dict/str input handling via `@dict_or_str`, config validation. **Holds the user-facing docstrings** (see "Docstring placement" below) |
| [valhalla/config.py](valhalla/config.py) | `get_config`, `get_help`, `parse_and_validate_config` |
| [valhalla/utils/__init__.py](valhalla/utils/__init__.py) | Re-exports `GraphId`, `GraphUtils`, tile helpers, DCT helpers |
| [valhalla/utils/graph_utils.py](valhalla/utils/graph_utils.py) | `GraphUtils(_GraphUtils)` wrapper |
| [valhalla/utils/decode_polyline.py](valhalla/utils/decode_polyline.py) | Polyline6 decoder (pure Python) |
| [valhalla/__main__.py](valhalla/__main__.py), [valhalla/_scripts.py](valhalla/_scripts.py) | CLI entry points (installed only in scikit-build-core wheel builds — see [CMakeLists.txt](CMakeLists.txt) `SKBUILD` branch) |

## .pyi Stubs

`nanobind_add_stub` in [CMakeLists.txt](CMakeLists.txt) generates `_valhalla.pyi`, `_graph_utils.pyi`, `predicted_speeds.pyi` directly into the source tree. They are **committed** and **refreshed on every build** — single source of truth is the compiled module (which in turn reads docstrings from the `.def(...)` calls).

`INCLUDE_PRIVATE` is set so the `_Actor` / `_GraphUtils` classes (underscore-prefixed) survive stubgen's default filter. The Python wrappers (`Actor`, `GraphUtils`) inherit from these and need them visible in the stub.

Install is straightforward: `install(FILES ${SRC}/.../X.pyi DESTINATION ...)` — pulls directly from the source tree, no build-dir intermediate. Ruff is told to skip `.pyi` files in both [`scripts/format.sh`](../../../scripts/format.sh) (`! -name "*.pyi"` in the find pattern) and [`pyproject.toml`](../../../pyproject.toml) (`extend-exclude`).

### Keeping stubs in sync

**Pre-commit hook** ([../../../.pre-commit-config.yaml](../../../.pre-commit-config.yaml), `python-stubs`) runs [`scripts/regenerate_python_stubs.sh`](../../../scripts/regenerate_python_stubs.sh) when any `src/bindings/python/src/*.{cc,h}` or `CMakeLists.txt` is staged. The script:
1. Rebuilds the `*_stub` targets in `BUILD_DIR` (default `build`, overridable by env var or first positional arg).
2. `git status --porcelain`s the three `.pyi` paths; exits 1 with the diff if anything changed.

**CI** runs the same script in the `arm_build` job ([.github/workflows/linux.yml](../../../.github/workflows/linux.yml)) right after `make`. The build already produces the stubs as part of `ALL`, so the script's `cmake --build` is a no-op rebuild; the `git status` check is what fails the job.

## Docstring Placement (Important — Non-Obvious)

Pyright/Pylance reads docstrings **statically** and does **not** walk the MRO. So docstrings on a C++-bound parent class (`_Actor`) do not surface on hover for the Python subclass (`Actor`) in VSCode, even though `help(Actor)` works at runtime.

The convention:
- **User-facing classes and methods** that have a Python wrapper → docstring as a string literal in the Python source (e.g., [valhalla/actor.py](valhalla/actor.py)). Pyright sees the literal, VSCode hover works.
- **Bindings without a Python wrapper** (currently: `_Actor.optimized_route`, exception classes, free functions in `_graph_utils` and `predicted_speeds`) → docstring stays in the C++ `.def(...)` / `nb::class_(...)` arg. The generated `.pyi` carries it.

Don't add C++ docstrings to methods that also have a Python wrapper with its own docstring — that duplicates and drifts.

## Add a new binding function

1. Add `.def(...)` to the relevant `.cc` (or a new `m.def(...)` for free functions).
2. If it gets a Python wrapper (e.g., `dict_or_str` handling), add it to the matching Python class and put the docstring there. Otherwise put the docstring in the `.def(...)`.
3. Rebuild — the `*_stub` target regenerates the `.pyi` automatically.
4. Commit both the `.cc` and the `.pyi`. CI will fail the PR if you forget the stub.

## Add a new module

1. `nanobind_add_module(<name> NB_STATIC STABLE_ABI [LTO] <sources>)` in [CMakeLists.txt](CMakeLists.txt). Pattern after `_valhalla`. Add it to the `foreach` that runs `nanobind_extension_abi3`, sets includes, links `valhalla` + `Python::SABIModule`.
2. Set `LIBRARY_OUTPUT_DIRECTORY` to the dir under `${CMAKE_CURRENT_BINARY_DIR}/valhalla/...` where it should live in the wheel.
3. Add `nanobind_add_stub(<name>_stub MODULE <name> OUTPUT ${SRC}/.../X.pyi PYTHON_PATH $<TARGET_FILE_DIR:<name>> DEPENDS <name> INCLUDE_PRIVATE)`.
4. `install(TARGETS <name> DESTINATION ${VALHALLA_INSTALL_PYDIR}/...)` + `install(FILES ${SRC}/.../X.pyi ...)`.
5. If the module has Python-facing classes, write a wrapper in `valhalla/...` and re-export from the right `__init__.py`.

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

Note: `valhalla` (the main C++ library) is built `POSITION_INDEPENDENT_CODE ON` when the python bindings are enabled, so each module can statically link against the right `.o`s. Even with `BUILD_SHARED_LIBS=Off` on the main library, the modules use it via PIC.

## Build / ABI Details

- **`STABLE_ABI`** on all three modules → abi3 wheels, one wheel per Python major. `wheel.py-api = "cp312"` in [pyproject.toml](../../../pyproject.toml).
- **`NB_STATIC`** → nanobind's runtime is statically linked into each module (no separate `libnanobind.so`).
- **`LTO`** enabled for `_valhalla` and `predicted_speeds`. **Disabled for `_graph_utils`** — `graph_id.cc` pulls in heavy `baldr/graphreader.h` templates that trigger a GCC 14 ARM64 LTO ICE (`internal compiler error: in get_token`). See the comment in [CMakeLists.txt](CMakeLists.txt) above the `_graph_utils` target.
- **`RPATH`** set to `$ORIGIN` (Linux) / `@loader_path` (macOS) so the `.abi3.so` finds the sibling `libvalhalla.so` inside the wheel.
- **`LIBRARY_OUTPUT_DIRECTORY`** places each module under `${BIN}/valhalla/...` so the build tree mirrors the install layout.

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

`ValhallaError` is created manually with `PyErr_NewExceptionWithDoc` (not `nb::exception`) so the translator can populate structured attributes (`code`, `http_code`, `message`, `http_message`) on the instance. See the `nb::register_exception_translator` block in [src/_valhalla.cc](src/_valhalla.cc). Don't switch to `nb::exception` without preserving these fields — downstream users read them.

## Wheels

`pyproject.toml` drives cibuildwheel. Linux uses a custom manylinux image. The `auditwheel` patch in [scripts/auditwheel_patch.py](scripts/auditwheel_patch.py) allows vendoring libraries that are also linked to libpython (which auditwheel normally refuses). See the `[tool.cibuildwheel.linux]` block.
