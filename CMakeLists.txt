# Minimal CMake configuration for Valhalla
#
# Builds libvalhalla and minimal collection of programs.
#
# This is NOT equivalent to the official Valhalla build configuration based on GNU Autotools.
# This is NOT suitable for building complete Valhalla suite.
# This is secondary build configuration provided for convenient development
# on Windows and using CMake-enabled IDEs.
#
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(valhalla LANGUAGES CXX C)

include(FindPkgConfig)
include(GNUInstallDirs)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

option(ENABLE_CCACHE "Speed up incremental rebuilds via ccache" ON)
option(ENABLE_DATA_TOOLS "Enable Valhalla data tools" ON)
option(ENABLE_SERVICES "Enable Valhalla services" ON)
option(ENABLE_PYTHON_BINDINGS "Enable Python bindings" ON)
option(ENABLE_COVERAGE "Build with coverage instrumentalisation" OFF)
option(ENABLE_SANITIZER "Use memory sanitizer for Debug build" OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)


## Get Valhalla version
file(STRINGS "valhalla/valhalla.h.in" version_lines REGEX "VALHALLA_VERSION_(MAJOR|MINOR|PATCH)")
foreach(line ${version_lines})
  if("${line}" MATCHES "(VALHALLA_VERSION_(MAJOR|MINOR|PATCH))[\t ]+([0-9]+)")
    set(${CMAKE_MATCH_1} ${CMAKE_MATCH_3})
  endif()
endforeach()
if(VALHALLA_VERSION_MAJOR)
  set(VERSION "${VALHALLA_VERSION_MAJOR}")
  if(VALHALLA_VERSION_MINOR)
    set(VERSION "${VERSION}.${VALHALLA_VERSION_MINOR}")
    if(VALHALLA_VERSION_PATCH)
      set(VERSION "${VERSION}.${VALHALLA_VERSION_PATCH}")
    endif()
  endif()
else()
  message(FATAL_ERROR "No Valhalla major version")
endif()


## Configure imported libraries
set(boost_components
  date_time
  filesystem
  iostreams
  program_options
  regex
  system)
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  list(APPEND boost_components zlib)
endif()
find_package(Boost 1.51 REQUIRED COMPONENTS ${boost_components})

find_package(CURL REQUIRED)
add_library(CURL::CURL INTERFACE IMPORTED)
set_target_properties(CURL::CURL PROPERTIES
  INTERFACE_LINK_LIBRARIES "${CURL_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${CURL_INCLUDE_DIR}"
  INTERFACE_COMPILE_DEFINITIONS CURL_STATICLIB)

find_package(Lua REQUIRED)
add_library(Lua::Lua INTERFACE IMPORTED)
set_target_properties(Lua::Lua PROPERTIES
  INTERFACE_LINK_LIBRARIES "${LUA_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${LUA_INCLUDE_DIR}")

find_package(ZLIB REQUIRED)
find_package(LZ4 REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(SpatiaLite)
find_package(GEOS)
find_package(Threads REQUIRED)
add_library(libprime_server INTERFACE IMPORTED)
if(ENABLE_SERVICES)
  pkg_check_modules(libprime_server REQUIRED libprime_server>=0.6.3)
  set_target_properties(libprime_server PROPERTIES
    INTERFACE_LINK_LIBRARIES "${libprime_server_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${libprime_server_INCLUDE_DIRS}"
    INTERFACE_COMPILE_DEFINITIONS HAVE_HTTP)
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

if(ENABLE_SANITIZER)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address")
endif()


## libvalhalla
add_subdirectory(src)


## Executable targets
function(get_source_path PATH NAME)
  if(EXISTS ${CMAKE_SOURCE_DIR}/src/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/${NAME}.cc PARENT_SCOPE)
  elseif(EXISTS ${CMAKE_SOURCE_DIR}/src/meili/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/meili/${NAME}.cc PARENT_SCOPE)
  elseif(EXISTS ${CMAKE_SOURCE_DIR}/src/mjolnir/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/mjolnir/${NAME}.cc PARENT_SCOPE)
  else()
    message(FATAL_ERROR "no source path for ${NAME}")
  endif()
endfunction()

## Valhalla programs
set(valhalla_programs valhalla_run_map_match valhalla_benchmark_loki valhalla_benchmark_skadi
  valhalla_run_isochrone valhalla_run_route valhalla_benchmark_adjacency_list valhalla_run_matrix
  valhalla_path_comparison valhalla_export_edges valhalla_pack_elevation)

## Valhalla data tools
set(valhalla_data_tools valhalla_build_statistics valhalla_ways_to_edges valhalla_validate_transit
  valhalla_benchmark_admins	valhalla_build_connectivity	valhalla_build_tiles
  valhalla_build_admins valhalla_build_transit valhalla_fetch_transit valhalla_query_transit
  valhalla_build_speeds  valhalla_associate_segments)

## Valhalla services
set(valhalla_services	valhalla_service valhalla_loki_worker	valhalla_odin_worker valhalla_thor_worker)

foreach(program ${valhalla_programs})
  get_source_path(path ${program})
  add_executable(${program} ${path})
  target_link_libraries(${program} Boost::program_options valhalla)
  install(TARGETS ${program} DESTINATION bin COMPONENT runtime)
endforeach()

if(ENABLE_DATA_TOOLS)
  foreach(program ${valhalla_data_tools})
    get_source_path(path ${program})
    add_executable(${program} ${path})
    target_link_libraries(${program} valhalla Boost::program_options)
    install(TARGETS ${program} DESTINATION bin COMPONENT runtime)
  endforeach()

  # Target-specific depedencies
  target_link_libraries(valhalla_build_admins GEOS::GEOS)
  target_sources(valhalla_build_statistics
    PUBLIC
      src/mjolnir/statistics.cc
      src/mjolnir/statistics_database.cc)
endif()

if(ENABLE_SERVICES)
  foreach(program ${valhalla_services})
    add_executable(${program} ${CMAKE_SOURCE_DIR}/src/${program}.cc)
    target_link_libraries(${program} valhalla Boost::program_options)
    install(TARGETS ${program} DESTINATION bin COMPONENT runtime)
  endforeach()
endif()

install(FILES COPYING ChangeLog
  DESTINATION "${CMAKE_INSTALL_DOCDIR}"
  COMPONENT runtime)

## Python bindings
if(ENABLE_PYTHON_BINDINGS)
  find_package(PythonInterp)
  find_package(PythonLibs)
  if(${PYTHON_VERSION_MAJOR} EQUAL 3)
    set(Boost_Python_lib "Boost::python${PYTHON_VERSION_MAJOR}")
    set(_Boost_PYTHON${PYTHON_VERSION_MAJOR}_HEADERS "boost/python.hpp")
    find_package(Boost COMPONENTS python${PYTHON_VERSION_MAJOR})
  else()
    set(Boost_Python_lib "Boost::python")
    find_package(Boost COMPONENTS python)
  endif()
  python_add_module(python_valhalla src/bindings/python/python.cc)
  set_target_properties(python_valhalla PROPERTIES
    OUTPUT_NAME valhalla
    LINK_LIBRARIES ${PYTHON_LIBRARIES}
    INCLUDE_DIRECTORIES ${PYTHON_INCLUDE_DIRS})
  target_link_libraries(python_valhalla valhalla ${Boost_Python_lib})
  set_target_properties(valhalla PROPERTIES POSITION_INDEPENDENT_CODE ON)

  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "
from distutils import sysconfig as sc
print(sc.get_python_lib(prefix='', plat_specific=True))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  install(TARGETS python_valhalla
    DESTINATION ${PYTHON_SITE_PACKAGES}
    COMPONENT python)
  install(FILES COPYING ChangeLog
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/python-valhalla"
    COMPONENT python)
endif()


## Tests TODO: move to own namespace
set(tests aabb2 actor admin astar attributes_controller datetime directededge
  distanceapproximator double_bucket_queue edgecollapser edge_elevation edgestatus ellipse encode
  enhancedtrippath factory graphid graphreader graphtile gridded_data grid_range_query grid_traversal
  json laneconnectivity linesegment2 location logging maneuversbuilder map_matcher_factory
  narrativebuilder narrative_dictionary navigator nodeinfo obb2 optimizer  point2 pointll
  polyline2 queue routing sample sequence serializers sign signs streetname streetnames streetnames_factory
  streetnames_us streetname_us tilehierarchy tiles traffic_matcher  turn util_midgard
  util_odin util_skadi vector2 verbal_text_formatter verbal_text_formatter_us verbal_text_formatter_us_co
  verbal_text_formatter_us_tx viterbi_search)

if(ENABLE_DATA_TOOLS)
  list(APPEND tests countryaccess edgeinfobuilder graphbuilder graphparser graphtilebuilder
    idtable mapmatch matrix names node_search refs search signinfo uniquenames utrecht)
endif()

if(ENABLE_SERVICES)
  list(APPEND tests loki_service skadi_service thor_service)
endif()

foreach(test ${tests})
  add_executable(${test} EXCLUDE_FROM_ALL	test/${test}.cc test/test.cc)
  set_target_properties(${test} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY unit_tests)
  target_link_libraries(${test} valhalla)
endforeach()

set(cost_tests autocost bicyclecost motorscootercost pedestriancost transitcost truckcost)
foreach(test ${cost_tests})
  add_executable(${test} EXCLUDE_FROM_ALL	src/sif/${test}.cc test/test.cc)
  set_target_properties(${test} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY unit_tests
    COMPILE_DEFINITIONS INLINE_TEST)
  target_link_libraries(${test} valhalla)
endforeach()

# Test-specific data, properties and dependencies
add_custom_command(OUTPUT ${CMAKE_SOURCE_DIR}/test/data/utrecht_tiles/0/003/196.gph
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles --inline-config '{"mjolnir":{"tile_dir":"test/data/utrecht_tiles","hierarchy":true,"shortcuts":true,"concurrency":4,"logging":{"type":""}}}' test/data/utrecht_netherlands.osm.pbf
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Building Utrecht Tiles..."
  DEPENDS valhalla_build_tiles ${CMAKE_SOURCE_DIR}/test/data/utrecht_netherlands.osm.pbf)
add_custom_target(utrecht_tiles DEPENDS ${CMAKE_SOURCE_DIR}/test/data/utrecht_tiles/0/003/196.gph)

set_target_properties(logging PROPERTIES COMPILE_DEFINITIONS LOGGING_LEVEL_ALL)

if(ENABLE_DATA_TOOLS)
  add_dependencies(mapmatch utrecht_tiles)
endif()

# Test run targets
foreach(test ${tests} ${cost_tests})
  add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/unit_tests/${test}.log
    COMMAND LOCPATH=locales /bin/bash -c -o pipefail "${CMAKE_BINARY_DIR}/unit_tests/${test} | tee ${CMAKE_BINARY_DIR}/unit_tests/${test}.log"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    DEPENDS ${test}
    VERBATIM)
  add_custom_target(run-${test} DEPENDS ${CMAKE_BINARY_DIR}/unit_tests/${test}.log)
endforeach()

string(REGEX REPLACE "([^;]+)" "run-\\1" test_targets "${tests};${cost_tests}")
add_custom_target(check DEPENDS ${test_targets})
add_custom_target(tests DEPENDS ${tests} ${cost_tests})

## Coverage report
if(ENABLE_COVERAGE)
  if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    message(WARNING "ENABLE_COVERAGE=ON only make sense with a Debug build")
  else()
    message(STATUS "Enabling coverage")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ftest-coverage -fprofile-arcs")
    target_link_libraries(valhalla gcov)

    find_program(LCOV_PATH NAMES lcov lcov.bat lcov.exe lcov.perl)
    find_program(GENHTML_PATH NAMES genhtml genhtml.perl genhtml.bat)
    if(NOT LCOV_PATH OR NOT GENHTML_PATH)
      message(FATAL_ERROR "no lcov or genhtml installed")
    endif()

    add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/coverage.info
      COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR} --base-directory ${CMAKE_SOURCE_DIR} --no-external --capture --output-file ${CMAKE_BINARY_DIR}/coverage.info --no-checksum
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      DEPENDS check)

    add_custom_target(coverage
      COMMAND ${GENHTML_PATH} --prefix ${CMAKE_BINARY_DIR} --output-directory ${CMAKE_BINARY_DIR}/coverage --title "Test Coverage" --legend --show-details coverage.info
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      DEPENDS ${CMAKE_BINARY_DIR}/coverage.info)
  endif()
endif()

## Packaging via CPack
include(CPackComponent)

string(TOLOWER "${CMAKE_PROJECT_NAME}" CPACK_PACKAGE_NAME)
set(CPACK_PACKAGE_VERSION_MAJOR ${VALHALLA_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VALHALLA_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VALHALLA_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}${CPACK_PACKAGE_VERSION_SUFFIX}")
set(CPACK_PACKAGE_CONTACT "Team Valhalla <valhalla@mapzen.com>")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE.md")
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

  message(STATUS "Configured deb packages of ${CMAKE_CXX_LIBRARY_ARCHITECTURE} build for ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
endif()

if(BUILD_SHARED_LIBS)
  set(CPACK_COMPONENTS_ALL "shared;runtime;python")
else()
  set(CPACK_COMPONENTS_ALL "development")
endif()

set(CPACK_PROJECT_CONFIG_FILE ${CMAKE_SOURCE_DIR}/cmake/CPackConfig.cmake)
set(CPACK_DEBIAN_PACKAGE_DEBUG OFF)
include(CPack)
