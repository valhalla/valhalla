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

if (ENABLE_SANITIZER)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address")
endif()


## Custom commands for generated files
find_program(XXD xxd)
if (NOT XXD)
  message(FATAL_ERROR "xxd executable not found")
endif()

## libvalhalla
add_subdirectory(proto ${CMAKE_BINARY_DIR}/valhalla/proto)
add_subdirectory(src/baldr)
add_subdirectory(src/loki)
add_subdirectory(src/meili)
add_subdirectory(src/midgard)
add_subdirectory(src/mjolnir)
add_subdirectory(src/odin)
add_subdirectory(src/sif)
add_subdirectory(src/skadi)
add_subdirectory(src/thor)
add_subdirectory(src/tyr)

message(STATUS "Generating  ${CMAKE_BINARY_DIR}/valhalla/valhalla.h")
configure_file(${CMAKE_SOURCE_DIR}/valhalla/valhalla.h.in ${CMAKE_BINARY_DIR}/valhalla/valhalla.h @ONLY)

message(STATUS "Generating  ${CMAKE_BINARY_DIR}/valhalla/config.h")
configure_file(${CMAKE_SOURCE_DIR}/valhalla/config.h.cmake ${CMAKE_BINARY_DIR}/valhalla/config.h @ONLY)

message(STATUS "Configuring valhalla library target")
set(valhalla_hdrs
  ${CMAKE_BINARY_DIR}/valhalla/valhalla.h
  ${CMAKE_SOURCE_DIR}/valhalla/exception.h
  ${CMAKE_SOURCE_DIR}/valhalla/worker.h)

set(valhalla_srcs
  ${CMAKE_SOURCE_DIR}/src/worker.cc)

add_library(valhalla
  ${valhalla_srcs}
  ${valhalla_hdrs})

if (MSVC)
  target_compile_definitions(valhalla PRIVATE "VC_EXTRALEAN;WIN32_LEAN_AND_MEAN;NOMINMAX;NOGDI")
endif()

target_include_directories(valhalla
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
    ${CMAKE_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/valhalla)

target_link_libraries(valhalla
  valhalla::mjolnir
  valhalla::baldr
  valhalla::tyr
  valhalla::midgard
  valhalla::meili
  valhalla::sif
  valhalla::skadi
  valhalla::protobuf)

if(NOT BUILD_SHARED_LIBS)
  add_custom_command(TARGET valhalla POST_BUILD
    COMMAND ${CMAKE_AR} rc $<TARGET_FILE:valhalla>
      $<TARGET_FILE:valhalla::baldr>
      $<TARGET_FILE:valhalla::loki>
      $<TARGET_FILE:valhalla::meili>
      $<TARGET_FILE:valhalla::midgard>
      $<TARGET_FILE:valhalla::mjolnir>
      $<TARGET_FILE:valhalla::odin>
      $<TARGET_FILE:valhalla::sif>
      $<TARGET_FILE:valhalla::skadi>
      $<TARGET_FILE:valhalla::thor>
      $<TARGET_FILE:valhalla::tyr>
      $<TARGET_FILE:valhalla::mjolnir>)
endif()

set_target_properties(valhalla PROPERTIES
  VERSION 2.4.9
  SOVERSION 2
  PUBLIC_HEADER "${valhalla_hdrs}")
install(TARGETS valhalla
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla)
# TODO: pkg-config
# configure_file(libvalhalla.pc.in libvalhalla.pc @ONLY) TODO
#install(FILES ${CMAKE_BINARY_DIR}/libvalhalla.pc
#    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)

## Executable targets
function(get_source_path PATH NAME)
  if(EXISTS ${CMAKE_SOURCE_DIR}/src/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/${NAME}.cc PARENT_SCOPE)
  elseif(EXISTS ${CMAKE_SOURCE_DIR}/src/meili/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/meili/${NAME}.cc PARENT_SCOPE)
  elseif(EXISTS ${CMAKE_SOURCE_DIR}/src/mjolnir/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/mjolnir/${NAME}.cc PARENT_SCOPE)
  else()
    message(ERROR "no source path for ${NAME}")
  endif()
endfunction()

## Valhalla programs
set(valhalla_programs valhalla_run_map_match valhalla_benchmark_loki valhalla_benchmark_skadi
  valhalla_run_isochrone valhalla_run_route valhalla_benchmark_adjacency_list valhalla_run_matrix
  valhalla_path_comparison valhalla_export_edges valhalla_pack_elevation)

## Valhalla data tools
set(valhalla_data_tools valhalla_ways_to_edges valhalla_validate_transit
  valhalla_benchmark_admins	valhalla_build_connectivity	valhalla_build_tiles
  valhalla_build_admins valhalla_build_transit valhalla_fetch_transit valhalla_query_transit
  valhalla_build_speeds  valhalla_associate_segments)

## Valhalla services
set(valhalla_services	valhalla_service valhalla_loki_worker	valhalla_odin_worker valhalla_thor_worker)

foreach(program ${valhalla_programs})
  get_source_path(path ${program})
  add_executable(${program} ${path})
  target_link_libraries(${program} Boost::program_options valhalla)
  install(TARGETS ${program} DESTINATION bin)
endforeach()

if(ENABLE_DATA_TOOLS)
  foreach(program ${valhalla_data_tools})
    get_source_path(path ${program})
    add_executable(${program} ${path})
    target_link_libraries(${program} valhalla GEOS::GEOS Boost::program_options)
    install(TARGETS ${program} DESTINATION bin)
  endforeach()

  add_executable(valhalla_build_statistics
    src/mjolnir/valhalla_build_statistics.cc
    src/mjolnir/statistics.cc
    src/mjolnir/statistics_database.cc)
  target_link_libraries(valhalla_build_statistics valhalla Boost::program_options)
  install(TARGETS valhalla_build_statistics DESTINATION bin)
endif()

if(ENABLE_SERVICES)
  foreach(program ${valhalla_services})
    add_executable(${program} ${CMAKE_SOURCE_DIR}/src/${program}.cc)
    target_link_libraries(${program} valhalla Boost::program_options)
    install(TARGETS ${program} DESTINATION bin)
  endforeach()
endif()


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
  if (NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    message(ERROR "ENABLE_COVERAGE=ON only make sense with a Debug build")
  endif()
  message(STATUS "Enabling coverage")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -ftest-coverage -fprofile-arcs")
  target_link_libraries(valhalla gcov)

  # TODO: add configuration of gcov tools
  add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/coverage.info
    COMMAND lcov --directory ${CMAKE_BINARY_DIR} --base-directory ${CMAKE_SOURCE_DIR} --no-external --capture --output-file ${CMAKE_BINARY_DIR}/coverage.info --no-checksum
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS check)

  add_custom_target(coverage
    COMMAND genhtml --prefix ${CMAKE_BINARY_DIR} --output-directory ${CMAKE_BINARY_DIR}/coverage --title "Test Coverage" --legend --show-details coverage.info
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS ${CMAKE_BINARY_DIR}/coverage.info)
endif()

# TODO: add packaging via CPack