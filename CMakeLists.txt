# Minimal CMake configuration for Valhalla
#
# Builds libvalhalla and minimal collection of programs.
#
# This is NOT equivalent to the official Valhalla build configuration based on GNU Autotools.
# This is NOT suitable for building complete Valhalla suite.
# This is secondary build configuration provided for convenient development
# on Windows and using CMake-enabled IDEs.
#
cmake_minimum_required(VERSION 3.12 FATAL_ERROR)
project(valhalla LANGUAGES CXX C)

include(FindPkgConfig)
include(GNUInstallDirs)

set(VALHALLA_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(VALHALLA_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR})
list(INSERT CMAKE_MODULE_PATH 0 ${VALHALLA_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ language version to use (default is 14)")
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
option(ENABLE_BENCHMARKS "Enable microbenchmarking" ON)
option(ENABLE_THREAD_SAFE_TILE_REF_COUNT "If ON uses shared_ptr as tile reference(i.e. it is thread safe)" OFF)
option(ENABLE_SINGLE_FILES_WERROR "Convert compiler warnings to errors for single files" ON)
# useful to workaround issues likes this https://stackoverflow.com/questions/24078873/cmake-generated-xcode-project-wont-compile
option(ENABLE_STATIC_LIBRARY_MODULES "If ON builds Valhalla modules as STATIC library targets" OFF)

set(LOGGING_LEVEL "" CACHE STRING "Logging level, default is INFO")
set_property(CACHE LOGGING_LEVEL PROPERTY STRINGS "NONE;ALL;ERROR;WARN;INFO;DEBUG;TRACE")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# colorize output
include(CheckCXXCompilerFlag)
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(COLOR_FLAG "-fdiagnostics-color=auto")
  check_cxx_compiler_flag("-fdiagnostics-color=auto" HAS_COLOR_FLAG)
  if(NOT HAS_COLOR_FLAG)
    set(COLOR_FLAG "")
  endif()
  # using GCC
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COLOR_FLAG}")
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
  message(STATUS "Configuring in release mode with debug flags")
elseif(CMAKE_BUILD_TYPE STREQUAL MinRelSize)
  message(STATUS "Configuring in release mode with minimized size")
elseif(CMAKE_BUILD_TYPE STREQUAL None)
  message(STATUS "Configuring without a mode, no optimization flags will be set")
else()
  message(FATAL_ERROR "Unrecognized build type. Use one of Debug, Release, RelWithDebInfo, MinRelSize, None")
endif()

function(create_source_groups prefix)
  foreach(file ${ARGN})
    get_filename_component(file "${file}" ABSOLUTE)
    string(FIND "${file}" "${PROJECT_BINARY_DIR}/" pos)
    if(pos EQUAL 0)
      source_group(TREE "${PROJECT_BINARY_DIR}/" PREFIX "Generated Files" FILES "${file}")
    else()
      source_group(TREE "${PROJECT_SOURCE_DIR}/" PREFIX "${prefix}" FILES "${file}")
    endif()
  endforeach()
endfunction()

if(ENABLE_CCACHE AND (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    message(STATUS "Using ccache to speed up incremental builds")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    set(ENV{CCACHE_CPP2} "true")
  endif()
endif()

include(cmake/SanitizerOptions.cmake)

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

## Dependencies
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)

if(NOT TARGET CURL::CURL)
  add_library(CURL::CURL INTERFACE IMPORTED)
  if(ENABLE_HTTP OR ENABLE_DATA_TOOLS)
    if(NOT CURL_FOUND)
      find_package(CURL REQUIRED)
    endif()
    set_target_properties(CURL::CURL PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${CURL_INCLUDE_DIR}"
      INTERFACE_COMPILE_DEFINITIONS CURL_STATICLIB)
    if(NOT MSVC)
      set_property(TARGET CURL::CURL APPEND PROPERTY INTERFACE_LINK_LIBRARIES "${CURL_LIBRARIES}")
    else()
      link_libraries(${CURL_LIBRARIES})
    endif()
  endif()
else()
  message(STATUS "Using curl from the outside")
endif()

if(NOT Protobuf_FOUND)
  find_package(Protobuf REQUIRED)
endif()

message(STATUS "Using pbf headers from ${PROTOBUF_INCLUDE_DIR}")
message(STATUS "Using pbf libs from ${PROTOBUF_LIBRARY}")
message(STATUS "Using pbf release libs from ${PROTOBUF_LIBRARY_RELEASE}")
message(STATUS "Using pbf debug libs from ${PROTOBUF_LIBRARY_DEBUG}")
if(TARGET protobuf::libprotobuf-lite)
  message(STATUS "Using pbf-lite")
endif()

# TODO: remove after switching to CMake >= 3.9
if(CMAKE_VERSION VERSION_LESS 3.9)
  if(NOT TARGET protobuf::libprotobuf-lite)
    add_library(protobuf::libprotobuf-lite UNKNOWN IMPORTED)
    set_target_properties(protobuf::libprotobuf-lite PROPERTIES
      FOLDER "Dependencies"
      INTERFACE_INCLUDE_DIRECTORIES "${PROTOBUF_INCLUDE_DIR}")

    if(EXISTS "${PROTOBUF_LIBRARY}")
      set_target_properties(protobuf::libprotobuf-lite PROPERTIES
        IMPORTED_LOCATION "${PROTOBUF_LIBRARY}")
    endif()
    if(EXISTS "${PROTOBUF_LIBRARY_RELEASE}")
      set_property(TARGET protobuf::libprotobuf-lite APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(protobuf::libprotobuf-lite PROPERTIES
        IMPORTED_LOCATION_RELEASE "${PROTOBUF_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${PROTOBUF_LIBRARY_DEBUG}")
      set_property(TARGET protobuf::libprotobuf-lite APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(protobuf::libprotobuf-lite PROPERTIES
        IMPORTED_LOCATION_DEBUG "${PROTOBUF_LIBRARY_DEBUG}")
    endif()
  endif()
endif()

# Allow linking against full or lite version of Protobuf library
# TODO: After switching to CMake 3.12+, replace with
#       $<TARGET_EXISTS:protobuf::libprotobuf
#       $<TARGET_EXISTS:protobuf::libprotobuf-lite
if(TARGET protobuf::libprotobuf-lite)
  set(valhalla_protobuf_targets protobuf::libprotobuf-lite)
elseif(TARGET protobuf::libprotobuf)
  set(valhalla_protobuf_targets protobuf::libprotobuf)
else()
  message(FATAL_ERROR "Required target protobuf::libprotobuf-lite or protobuf::libprotobuf is not defined")
endif()

add_library(libprime_server INTERFACE IMPORTED)
if(ENABLE_SERVICES)
  pkg_check_modules(libprime_server REQUIRED libprime_server>=0.6.3)
  # workaround for https://gitlab.kitware.com/cmake/cmake/issues/15804
  find_library(libprime_server_LIBRARY
    NAME ${libprime_server_LIBRARIES}
    HINTS ${libprime_server_LIBRARY_DIRS})
  set_target_properties(libprime_server PROPERTIES
    INTERFACE_LINK_LIBRARIES "${libprime_server_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${libprime_server_INCLUDE_DIRS}"
    INTERFACE_COMPILE_DEFINITIONS HAVE_HTTP)
endif()

## Mjolnir and associated executables
if(ENABLE_DATA_TOOLS)
  add_compile_definitions(DATA_TOOLS)
  find_package(Boost 1.51 REQUIRED COMPONENTS program_options)
  find_package(SQLite3 REQUIRED)
  find_package(SpatiaLite REQUIRED)
  find_package(LuaJIT)
  add_library(Lua::Lua INTERFACE IMPORTED)
  set_target_properties(Lua::Lua PROPERTIES
    INTERFACE_LINK_LIBRARIES "${LUA_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${LUA_INCLUDE_DIR}")
## Top level executables
elseif(ENABLE_TOOLS)
  find_package(Boost 1.51 REQUIRED COMPONENTS program_options)
## Header only boost for the library without mjolnir
elseif(NOT Boost_FOUND)
  find_package(Boost 1.51 REQUIRED)
endif()
add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)

if (ENABLE_THREAD_SAFE_TILE_REF_COUNT)
 add_definitions(-DENABLE_THREAD_SAFE_TILE_REF_COUNT)
endif ()

## libvalhalla
add_subdirectory(src)

## Python bindings
if(ENABLE_PYTHON_BINDINGS)
  # favors Py3, falls back to Py2
  find_package(Python COMPONENTS Development Interpreter)
  if (Python_FOUND)
    add_subdirectory(src/bindings/python)
    install(FILES COPYING CHANGELOG.md
      DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/python-valhalla"
      COMPONENT python)
  else()
    set(ENABLE_PYTHON_BINDINGS OFF)
    message(WARNING "Skipping Python bindings... Install a development Python version including headers to compile the bindings.")
  endif()
endif()

## Executable targets
function(get_source_path PATH NAME)
  if(EXISTS ${VALHALLA_SOURCE_DIR}/src/${NAME}.cc)
    set(${PATH} ${VALHALLA_SOURCE_DIR}/src/${NAME}.cc PARENT_SCOPE)
  elseif(EXISTS ${VALHALLA_SOURCE_DIR}/src/meili/${NAME}.cc)
    set(${PATH} ${VALHALLA_SOURCE_DIR}/src/meili/${NAME}.cc PARENT_SCOPE)
  elseif(EXISTS ${VALHALLA_SOURCE_DIR}/src/mjolnir/${NAME}.cc)
    set(${PATH} ${VALHALLA_SOURCE_DIR}/src/mjolnir/${NAME}.cc PARENT_SCOPE)
  else()
    message(FATAL_ERROR "no source path for ${NAME}")
  endif()
endfunction()

## Valhalla programs
set(valhalla_programs valhalla_run_map_match valhalla_benchmark_loki valhalla_benchmark_skadi
  valhalla_run_isochrone valhalla_run_route valhalla_benchmark_adjacency_list valhalla_run_matrix
  valhalla_path_comparison valhalla_export_edges valhalla_expand_bounding_box valhalla_service)

## Valhalla data tools
set(valhalla_data_tools valhalla_build_statistics valhalla_ways_to_edges valhalla_validate_transit
  valhalla_benchmark_admins valhalla_build_connectivity	valhalla_build_tiles valhalla_build_admins
  valhalla_convert_transit valhalla_fetch_transit valhalla_query_transit valhalla_add_predicted_traffic
  valhalla_assign_speeds)

## Valhalla services
set(valhalla_services valhalla_loki_worker valhalla_odin_worker valhalla_thor_worker)

if(ENABLE_TOOLS)
  foreach(program ${valhalla_programs})
    get_source_path(path ${program})
    add_executable(${program} ${path})
    set_target_properties(${program} PROPERTIES FOLDER "Tools")
    create_source_groups("Source Files" ${path})
    target_link_libraries(${program} Boost::program_options valhalla)
    if(MSVC)
      target_link_libraries(${program} ${CURL_LIBRARIES})
    endif()
    install(TARGETS ${program} DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime)
  endforeach()
endif()

if(ENABLE_DATA_TOOLS)
  foreach(program ${valhalla_data_tools})
    get_source_path(path ${program})
    add_executable(${program} ${path})
    create_source_groups("Source Files" ${path})
    set_target_properties(${program} PROPERTIES FOLDER "Data Tools")
    target_link_libraries(${program} valhalla Boost::program_options)
    if (LUAJIT_FOUND AND APPLE)
      # Using LuaJIT on macOS requires a couple of extra linker flags
      target_link_options(${program} PUBLIC -pagezero_size 10000 -image_base 100000000)
    endif()
    if(MSVC)
      target_link_libraries(${program} ${CURL_LIBRARIES})
    endif()
    install(TARGETS ${program} DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime)
  endforeach()

  # Target-specific depedencies
  find_package(GEOS)
  target_link_libraries(valhalla_build_admins GEOS::GEOS)
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
    target_link_libraries(${program} valhalla Boost::program_options)
    install(TARGETS ${program} DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime)
  endforeach()
endif()

install(
  FILES
    scripts/valhalla_build_config
    scripts/valhalla_build_elevation
    scripts/valhalla_build_transit
    scripts/valhalla_build_timezones
  DESTINATION "${CMAKE_INSTALL_BINDIR}"
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
  COMPONENT runtime)

install(FILES COPYING CHANGELOG.md
  DESTINATION "${CMAKE_INSTALL_DOCDIR}"
  COMPONENT runtime)

if(ENABLE_TESTS)
  add_subdirectory(test)
endif()

# NOTE(mookerji): Windows CI seems to break on the gbench build, so shelve Win32 support for now.
if(ENABLE_BENCHMARKS AND ENABLE_DATA_TOOLS AND NOT WIN32)
  add_subdirectory(bench)
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
  set(CPACK_COMPONENT_PYHON_DESCRIPTION "OpenStreetMap Routing Python Bindings
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
