# Minimal CMake configuration for Valhalla
#
# Builds libvalhalla and minimal collection of programs.
#
# This is NOT equivalent to the official Valhalla build configuration based on GNU Autotools.
# This is NOT suitable for building complete Valhalla suite.
# This is secondary build configuration provided for convenient development
# on Windows and using CMake-enabled IDEs.
#
cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(valhalla LANGUAGES CXX C)

include(FindPkgConfig)
include(GNUInstallDirs)

set(VALHALLA_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(VALHALLA_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR})
list(INSERT CMAKE_MODULE_PATH 0 ${VALHALLA_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ language version to use (default is 17)")
option(ENABLE_TOOLS "Enable Valhalla tools" ON)
option(ENABLE_DATA_TOOLS "Enable Valhalla data tools" ON)
option(ENABLE_SERVICES "Enable Valhalla services" ON)
option(ENABLE_HTTP "Enable the use of CURL" ON)
option(ENABLE_PYTHON_BINDINGS "Enable Python bindings" ON)
option(ENABLE_CCACHE "Speed up incremental rebuilds via ccache" ON)
option(ENABLE_COVERAGE "Build with coverage instrumentalisation" OFF)
option(ENABLE_COMPILER_WARNINGS "Build with compiler warnings" OFF)
option(ENABLE_SANITIZERS "Use all the integrated sanitizers for Debug build" OFF)
option(ENABLE_ADDRESS_SANITIZER "Use memory sanitizer for Debug build" OFF)
option(ENABLE_UNDEFINED_SANITIZER "Use UB sanitizer for Debug build" OFF)
option(ENABLE_TESTS "Enable Valhalla tests" ON)
option(ENABLE_WERROR "Convert compiler warnings to errors. Requires ENABLE_COMPILER_WARNINGS=ON to take effect" OFF)
option(ENABLE_THREAD_SAFE_TILE_REF_COUNT "If ON uses shared_ptr as tile reference(i.e. it is thread safe)" OFF)
option(ENABLE_SINGLE_FILES_WERROR "Convert compiler warnings to errors for single files" ON)
option(PREFER_EXTERNAL_DEPS "Whether to use internally vendored headers or find the equivalent external package" OFF)
# useful to workaround issues likes this https://stackoverflow.com/questions/24078873/cmake-generated-xcode-project-wont-compile
option(ENABLE_STATIC_LIBRARY_MODULES "If ON builds Valhalla modules as STATIC library targets" OFF)
option(ENABLE_GDAL "Whether to include GDAL; currently only used for raster serialization of isotile grid" ON)

set(LOGGING_LEVEL "" CACHE STRING "Logging level, default is INFO")
set_property(CACHE LOGGING_LEVEL PROPERTY STRINGS "NONE;ALL;ERROR;WARN;INFO;DEBUG;TRACE")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# colorize output
include(CheckCXXCompilerFlag)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  check_cxx_compiler_flag("-fdiagnostics-color=auto" HAS_COLOR_FLAG)
  if(HAS_COLOR_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COLOR_FLAG}")
  endif()
endif()

# Explicitly set the build type to Release if no other type is specified
# on the command line.  Without this, cmake defaults to an unoptimized,
# non-debug build, which almost nobody wants.
if(NOT MSVC_IDE) # TODO: May need to be extended for Xcode, CLion, etc.
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type specified, defaulting to Release")
    set(CMAKE_BUILD_TYPE Release)
  endif()
endif()

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  message(STATUS "Configuring in debug mode")
elseif(CMAKE_BUILD_TYPE STREQUAL Release)
  message(STATUS "Configuring in release mode")
elseif(CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
  message(STATUS "Configuring in release mode with debug symbols")
elseif(CMAKE_BUILD_TYPE STREQUAL MinRelSize)
  message(STATUS "Configuring in release mode with minimized size")
elseif(CMAKE_BUILD_TYPE STREQUAL None)
  message(STATUS "Configuring without a mode, no optimization flags will be set")
else()
  message(FATAL_ERROR "Unrecognized build type. Use one of Debug, Release, RelWithDebInfo, MinRelSize, None")
endif()

if(ENABLE_CCACHE AND (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    message(STATUS "Using ccache to speed up incremental builds")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    set(ENV{CCACHE_CPP2} "true")
  endif()
endif()

include(ValhallaSanitizerOptions)
include(ValhallaSourceGroups)

# We use pkg-config for (almost) all dependencies:
# - CMake Find* modules are versioned with CMake, not the packages, see protobuf > 21.12
# - it's more trivial to use custom installations via PKG_CONFIG_PATH env var
pkg_check_modules(ZLIB REQUIRED IMPORTED_TARGET zlib)
pkg_check_modules(LZ4 REQUIRED IMPORTED_TARGET liblz4)

# cURL
set(curl_targets "")
if (ENABLE_HTTP OR ENABLE_DATA_TOOLS)
  pkg_check_modules(CURL REQUIRED IMPORTED_TARGET libcurl)
  if (CURL_FOUND)
    set(curl_targets PkgConfig::CURL)
    target_compile_definitions(PkgConfig::CURL INTERFACE ENABLE_HTTP)
  else()
    message(FATAL_ERROR "ENABLE_HTTP=ON, but cURL not found...")
  endif()
endif()

# prefer CONFIG mode over MODULE mode, which versions configuration on the package, not CMake
# NOTE: this is only supported for cmake >= 3.15, but shouldn't be a problem in real life
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

# Boost has no .pc file..
find_package(Boost 1.71 REQUIRED)
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)
add_definitions(-DBOOST_ALLOW_DEPRECATED_HEADERS)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# resolve vendored libraries
set(date_include_dir ${VALHALLA_SOURCE_DIR}/third_party/date/include)
set(rapidjson_include_dir ${CMAKE_SOURCE_DIR}/third_party/rapidjson/include)
set(robinhoodhashing_include_dir ${CMAKE_SOURCE_DIR}/third_party/robin-hood-hashing/src/include)
set(cxxopts_include_dir ${VALHALLA_SOURCE_DIR}/third_party/cxxopts/include)
set(dirent_include_dir ${CMAKE_SOURCE_DIR}/third_party/dirent/include)
if (PREFER_EXTERNAL_DEPS)
  # date
  find_package(date QUIET)
  if (date_FOUND)
    get_target_property(date_include_dir date::date INTERFACE_INCLUDE_DIRECTORIES)
  else()
    message(WARNING "No date found in system libraries, using vendored date...")
  endif()
  # rapidjson
  find_package(RapidJSON QUIET)
  if (RapidJSON_FOUND)
    get_target_property(rapidjson_include_dir rapidjson INTERFACE_INCLUDE_DIRECTORIES)
  else()
    message(WARNING "No RapidJSON found in system libraries, using vendored RapidJSON...")
  endif()
  # robin-hood-hashing
  find_package(robin_hood QUIET)
  if (robin_hood_FOUND)
    get_target_property(robinhoodhashing_include_dir robin_hood::robin_hood INTERFACE_INCLUDE_DIRECTORIES)
  else()
    message(WARNING "No robin_hood found in system libraries, using vendored robin_hood...")
  endif()
  # cxxopts
  if (ENABLE_DATA_TOOLS OR ENABLE_TOOLS)
    find_package(cxxopts QUIET)
    if (cxxopts_FOUND)
      get_target_property(cxxopts_include_dir cxxopts::cxxopts INTERFACE_INCLUDE_DIRECTORIES)
    else()
      message(WARNING "No cxxopts found in system libraries, using vendored cxxopts...")
    endif()
  endif()
  # dirent
  if (WIN32)
    find_path(dirent_include_dir dirent.h REQUIRED)
    if (dirent_include_dir-NOTFOUND)
      message(WARNING "No dirent.h found in system headers, using vendored dirent.h...")
    endif()
  endif()
endif()

# Protobuf is non-trivial to include via pkg-config, pkg_check_modules has no way to check
# for protoc location in a platform agnostic manner
# newer protobuf versions require a compat bool
set(protobuf_MODULE_COMPATIBLE ON CACHE BOOL "")
find_package(Protobuf REQUIRED)
# and turn it off again
message(STATUS "Using protoc from ${Protobuf_PROTOC_EXECUTABLE}")
message(STATUS "Using pbf headers from ${Protobuf_INCLUDE_DIRS}")
message(STATUS "Using pbf libs from ${PROTOBUF_LIBRARIES}")
if(TARGET protobuf::libprotobuf-lite)
  message(STATUS "Using pbf-lite")
endif()
if(TARGET protobuf::libprotobuf-lite)
  set(valhalla_protobuf_targets protobuf::libprotobuf-lite)
elseif(TARGET protobuf::libprotobuf)
  set(valhalla_protobuf_targets protobuf::libprotobuf)
else()
  message(FATAL_ERROR "Required target protobuf::libprotobuf-lite or protobuf::libprotobuf is not defined")
endif()

# unless you said you didnt want gdal we try to turn it on, if we cant we tell you
# we don't need it to build tiles (yet)
set(GDAL_TARGET "")
if (ENABLE_GDAL AND (ENABLE_SERVICES OR ENABLE_TOOLS OR ENABLE_PYTHON_BINDINGS))
  find_package(GDAL QUIET)
  if (GDAL_FOUND)
    set(GDAL_TARGET GDAL::GDAL)
    add_compile_definitions(ENABLE_GDAL)
    message(STATUS "Gdal support is enabled")
  else()
    message(WARNING "Unable to enable gdal support")
  endif()
endif()

set(CMAKE_FIND_PACKAGE_PREFER_CONFIG OFF)

# libprime_server
# see https://gitlab.kitware.com/cmake/cmake/-/issues/19467
set(libprime_server_targets "")
if(ENABLE_SERVICES)
  pkg_check_modules(libprime_server IMPORTED_TARGET libprime_server>=0.6.3)
  if (libprime_server_FOUND)
    target_compile_definitions(PkgConfig::libprime_server INTERFACE ENABLE_SERVICES)
    set(libprime_server_targets PkgConfig::libprime_server)
  else()
    set(ENABLE_SERVICES OFF)
    message(FATAL_ERROR "ENABLE_SERVICES=ON but prime_server not found...")
  endif()
endif()


## Mjolnir and associated executables
if(ENABLE_DATA_TOOLS)
  add_compile_definitions(DATA_TOOLS)
  # keep sqlite3 with cmake find_package(), as OSX needs our own FindSqlite3.cmake
  # otherwise the system sqlite3 is found which doesn't allow for spatialite
  find_package(SQLite3 REQUIRED)
  pkg_check_modules(SpatiaLite REQUIRED IMPORTED_TARGET spatialite)
  pkg_check_modules(LuaJIT REQUIRED IMPORTED_TARGET luajit)
endif()

if (ENABLE_THREAD_SAFE_TILE_REF_COUNT)
 add_definitions(-DENABLE_THREAD_SAFE_TILE_REF_COUNT)
endif ()

## libvalhalla
add_subdirectory(src)

## Python bindings
if(ENABLE_PYTHON_BINDINGS)
  # python is fine for cmake as well
  find_package(Python COMPONENTS Development Interpreter)
  if (Python_FOUND)
    add_subdirectory(src/bindings/python)
    install(FILES COPYING CHANGELOG.md
      DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/python-valhalla"
      COMPONENT python)
  else()
    set(ENABLE_PYTHON_BINDINGS OFF)
    message(WARNING "Python development version not found, skipping Python bindings...")
  endif()
endif()

## Executable targets

## Valhalla programs
set(valhalla_programs valhalla_run_map_match valhalla_benchmark_loki valhalla_benchmark_skadi
  valhalla_run_isochrone valhalla_run_route valhalla_benchmark_adjacency_list valhalla_run_matrix
  valhalla_path_comparison valhalla_export_edges valhalla_expand_bounding_box valhalla_service)

## Valhalla data tools
set(valhalla_data_tools valhalla_build_statistics valhalla_ways_to_edges valhalla_validate_transit
  valhalla_benchmark_admins valhalla_build_connectivity	valhalla_build_tiles valhalla_build_admins
  valhalla_convert_transit valhalla_ingest_transit valhalla_query_transit valhalla_add_predicted_traffic
  valhalla_assign_speeds valhalla_add_elevation valhalla_build_landmarks valhalla_add_landmarks)

## Valhalla services
set(valhalla_services valhalla_loki_worker valhalla_odin_worker valhalla_thor_worker)

if(ENABLE_TOOLS)
  foreach(program ${valhalla_programs})
    get_source_path(path ${program})
    add_executable(${program} ${path})
    set_target_properties(${program} PROPERTIES FOLDER "Tools")
    create_source_groups("Source Files" ${path})
    target_link_libraries(${program} valhalla)
    target_include_directories(${program} PRIVATE ${cxxopts_include_dir})
    install(TARGETS ${program} DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime)
  endforeach()
endif()

if(ENABLE_DATA_TOOLS)
  foreach(program ${valhalla_data_tools})
    get_source_path(path ${program})
    add_executable(${program} ${path})
    create_source_groups("Source Files" ${path})
    set_target_properties(${program} PROPERTIES FOLDER "Data Tools")
    target_include_directories(${program} PRIVATE ${cxxopts_include_dir})
    target_link_libraries(${program} valhalla)
    if (LuaJIT_FOUND AND APPLE AND CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
      # Using LuaJIT on macOS on Intel processors requires a couple of extra linker flags
      target_link_options(${program} PUBLIC -pagezero_size 10000 -image_base 100000000)
    endif()
    install(TARGETS ${program} DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime)
  endforeach()

  # Target-specific dependencies
  pkg_check_modules(GEOS REQUIRED IMPORTED_TARGET geos)
  target_link_libraries(valhalla_build_admins PkgConfig::GEOS)
  target_sources(valhalla_build_statistics
    PUBLIC
      ${VALHALLA_SOURCE_DIR}/src/mjolnir/statistics.cc
      ${VALHALLA_SOURCE_DIR}/src/mjolnir/statistics_database.cc)
endif()

if(ENABLE_SERVICES)
  foreach(program ${valhalla_services})
    add_executable(${program} src/${program}.cc)
    create_source_groups("Source Files" src/${program}.cc)
    set_target_properties(${program} PROPERTIES FOLDER "Services")
    target_link_libraries(${program} valhalla)
    install(TARGETS ${program} DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime)
  endforeach()
endif()

# add the scripts to the build folder as well
foreach(script valhalla_build_config valhalla_build_elevation
  valhalla_build_extract valhalla_build_timezones)
  configure_file(${VALHALLA_SOURCE_DIR}/scripts/${script} ${CMAKE_BINARY_DIR}/${script} COPYONLY)
  
  install(
    FILES
      scripts/${script}
    DESTINATION "${CMAKE_INSTALL_BINDIR}"
    PERMISSIONS
      OWNER_READ OWNER_WRITE OWNER_EXECUTE
      GROUP_READ GROUP_EXECUTE
      WORLD_READ WORLD_EXECUTE
      COMPONENT runtime
    )
endforeach()

install(FILES COPYING CHANGELOG.md
  DESTINATION "${CMAKE_INSTALL_DOCDIR}"
  COMPONENT runtime)

# install third_party
install(DIRECTORY ${rapidjson_include_dir}/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/valhalla/third_party")
install(DIRECTORY ${date_include_dir}/
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/valhalla/third_party")
if (WIN32)
  install(DIRECTORY ${dirent_include_dir}/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/valhalla/third_party")
endif()

if(ENABLE_TESTS)
  add_subdirectory(test)
endif()

## Coverage report targets
if(ENABLE_COVERAGE)
  find_program(GENHTML_PATH NAMES genhtml genhtml.perl genhtml.bat)
  if(NOT GENHTML_PATH)
    message(FATAL_ERROR "no genhtml installed")
  endif()

  set(FASTCOV_PATH ${VALHALLA_SOURCE_DIR}/third_party/fastcov/fastcov.py)
  add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/coverage.info
    COMMAND ${FASTCOV_PATH} -d . --exclude /usr/ third_party/ ${CMAKE_CURRENT_BINARY_DIR}/ --lcov -o coverage.info
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS check)

  add_custom_target(coverage
    COMMAND ${GENHTML_PATH} --prefix ${CMAKE_CURRENT_BINARY_DIR} --output-directory coverage --title "Test Coverage" --legend --show-details coverage.info
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/coverage.info)
  set_target_properties(coverage PROPERTIES FOLDER "Tests")
endif()

## Packaging via CPack
include(CPackComponent)

string(TOLOWER "${CMAKE_PROJECT_NAME}" CPACK_PACKAGE_NAME)
set(CPACK_PACKAGE_VERSION_MAJOR ${VALHALLA_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VALHALLA_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VALHALLA_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}${CPACK_PACKAGE_VERSION_SUFFIX}")
set(CPACK_PACKAGE_CONTACT "Team Valhalla <valhalla@mapzen.com>")
set(CPACK_RESOURCE_FILE_LICENSE "${VALHALLA_SOURCE_DIR}/LICENSE.md")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "OpenStreetMap Routing API
 A set of routing APIs designed around OSM map data using
 dynamic costing and a tiled data structure")
  set(CPACK_COMPONENT_PYTHON_DESCRIPTION "OpenStreetMap Routing Python Bindings
 A set routing APIs designed around OSM map data using
 dynamic costing and a tiled data structure and
 accompanying tools and services used to analyse and
 compute routes using those APIs")
set(CPACK_STRIP_FILES TRUE)
set(CPACK_SOURCE_PACKAGE_FILE_NAME "libvalhalla")

if(${CPACK_GENERATOR} MATCHES "^DEB$")
  set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/valhalla/")
  set(CPACK_DEBIAN_FILE_NAME "DEB-DEFAULT")
  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

  set(CPACK_DEBIAN_SHARED_PACKAGE_NAME "libvalhalla0")
  set(CPACK_DEBIAN_SHARED_PACKAGE_SECTION "contrib/libs")

  set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_NAME "libvalhalla-dev")
  set(CPACK_DEBIAN_DEVELOPMENT_PACKAGE_DEPENDS "libvalhalla0 (= ${CPACK_PACKAGE_VERSION})")

  set(CPACK_DEBIAN_RUNTIME_PACKAGE_NAME "valhalla-bin")
  set(CPACK_DEBIAN_RUNTIME_PACKAGE_SECTION "contrib/misc")
  set(CPACK_DEBIAN_RUNTIME_PACKAGE_DEPENDS "libvalhalla0 (= ${CPACK_PACKAGE_VERSION})")

  set(CPACK_DEBIAN_PYTHON_PACKAGE_NAME "python-valhalla")
  set(CPACK_DEBIAN_PYTHON_PACKAGE_SECTION "python")
  set(CPACK_DEBIAN_PYTHON_PACKAGE_DEPENDS "libvalhalla0 (= ${CPACK_PACKAGE_VERSION})")

  if("${CMAKE_CXX_LIBRARY_ARCHITECTURE}" MATCHES "arm-linux-gnueabihf")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE armhf)
  endif()

  message(STATUS "Configured deb packages of ${CMAKE_CXX_LIBRARY_ARCHITECTURE} build for ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
endif()

if(BUILD_SHARED_LIBS)
  set(CPACK_COMPONENTS_ALL "shared;runtime;python")
else()
  set(CPACK_COMPONENTS_ALL "development")
endif()

set(CPACK_PROJECT_CONFIG_FILE ${VALHALLA_SOURCE_DIR}/cmake/CPackConfig.cmake)
set(CPACK_DEBIAN_PACKAGE_DEBUG OFF)
include(CPack)

configure_file(cmake/uninstall.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/valhalla_uninstall.cmake @ONLY)
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/valhalla_uninstall.cmake)
