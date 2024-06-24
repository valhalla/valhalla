### Build Configuration (all platforms)

Valhalla uses CMake as build system. When compiling with `gcc` (GNU Compiler Collection), currently version 5 or newer is supported.

Important build options include:

| Option | Behavior |
|--------|----------|
| `-DENABLE_TOOLS` (`On`/`Off`) | Build `valhalla_service` and other utilities (defaults to on)|
| `-DENABLE_DATA_TOOLS` (`On`/`Off`) | Build the data preprocessing tools (defaults to on)|
| `-DENABLE_HTTP` (`On`/`Off`) | Build with `curl` support (defaults to on)|
| `-DENABLE_PYTHON_BINDINGS` (`On`/`Off`) | Build the python bindings (defaults to on)|
| `-DENABLE_SERVICES` (`On` / `Off`) | Build the HTTP service (defaults to on)|
| `-DENABLE_THREAD_SAFE_TILE_REF_COUNT` (`ON` / `OFF`) | If ON uses shared_ptr as tile reference (i.e. it is thread safe, defaults to off)|
| `-DENABLE_CCACHE` (`On` / `Off`) | Speed up incremental rebuilds via ccache (defaults to on)|
| `-DENABLE_BENCHMARKS` (`On` / `Off`) | Enable microbenchmarking (defaults to on)|
| `-DENABLE_TESTS` (`On` / `Off`) | Enable Valhalla tests (defaults to on)|
| `-DENABLE_COVERAGE` (`On` / `Off`) | Build with coverage instrumentalisation (defaults to off)|
| `-DBUILD_SHARED_LIBS` (`On` / `Off`) | Build static or shared libraries (defaults to off)|
| `-DENABLE_STATIC_LIBRARY_MODULES` (`On` / `Off`) | If ON builds Valhalla modules as STATIC library targets (defaults to off)|
| `-DENABLE_COMPILER_WARNINGS` (`ON` / `OFF`) | Build with common compiler warnings (defaults to off)|
| `-DENABLE_SINGLE_FILES_WERROR` (`ON` / `OFF`) | Convert compiler warnings to errors for a (growing) selection of files (defaults to on)|
| `-DENABLE_WERROR` (`ON` / `OFF`) | Treat compiler warnings as errors  (defaults to off). Requires `-DENABLE_COMPILER_WARNINGS=ON` to take effect.|
| `-DENABLE_SANITIZERS` (`ON` / `OFF`) | Build with all the integrated sanitizers (defaults to off).|
| `-DENABLE_ADDRESS_SANITIZER` (`ON` / `OFF`) | Build with address sanitizer (defaults to off).|
| `-DENABLE_UNDEFINED_SANITIZER` (`ON` / `OFF`) | Build with undefined behavior sanitizer (defaults to off).|
| `-DPREFER_SYSTEM_DEPS` (`ON` / `OFF`) | Whether to use internally vendored headers or find the equivalent external package (defaults to off).|
| `-DENABLE_GDAL` (`ON` / `OFF`) | Whether to include GDAL as a dependency (used for GeoTIFF serialization of isochrone grid) (defaults to off).|
