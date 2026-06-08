cmake_minimum_required(VERSION 3.26 FATAL_ERROR)

include(ValhallaVersion)

if (PREFER_EXTERNAL_DEPS)
  find_package(nanobind QUIET CONFIG)
  if (NOT TARGET nanobind::nanobind)
    message(WARNING "No nanobind found in system libraries, using vendored nanobind...")
  endif()
endif()

if (NOT TARGET nanobind::nanobind)
  add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/nanobind ${CMAKE_BINARY_DIR}/third_party/nanobind EXCLUDE_FROM_ALL)
endif()

message(STATUS "Installing python modules to ${Python_SITEARCH}")

# See: https://github.com/actions/runner/issues/2508
# See: https://www.mail-archive.com/ptxdist@pengutronix.de/msg24413.html
nanobind_add_module(_valhalla
  NB_STATIC
  STABLE_ABI
  ${CMAKE_CURRENT_SOURCE_DIR}/src/_valhalla.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/actor.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/exceptions.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/baldr/graph_id.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/baldr/utils/graph_reader.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/baldr/utils/graph_tile.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/baldr/utils/predicted_speeds.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/midgard/utils/polyline.cc)

# force abi3.so extension
nanobind_extension_abi3(_valhalla)

target_include_directories(_valhalla PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../shared)
target_link_libraries(_valhalla PRIVATE
  valhalla
  Python::SABIModule
  $<$<BOOL:${ENABLE_COVERAGE}>:gcov>)

if(APPLE)
  set_target_properties(_valhalla PROPERTIES INSTALL_RPATH "@loader_path")
elseif(UNIX AND NOT APPLE)
  set_target_properties(_valhalla PROPERTIES INSTALL_RPATH "\$ORIGIN")
endif()

set_target_properties(_valhalla PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla)
set_target_properties(valhalla PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Generate Python type stubs (.pyi) from the compiled extension modules.
# Stubs are committed to the source tree and refreshed on every build;
# INCLUDE_PRIVATE keeps "_"-prefixed members in the output; the POST_BUILD
# step prepends a "machine-generated" banner via an idempotent script.
#
# Skipped under ENABLE_SANITIZERS: nanobind_add_stub imports the compiled
# module via a vanilla python3, which can't load an ASan-instrumented .so
if (NOT ENABLE_SANITIZERS)
  function(valhalla_python_stub module rel_pyi)
    set(_pyi ${CMAKE_CURRENT_SOURCE_DIR}/${rel_pyi})
    nanobind_add_stub(${module}_stub
      MODULE ${module}
      OUTPUT ${_pyi}
      PYTHON_PATH $<TARGET_FILE_DIR:${module}>
      DEPENDS ${module}
      INCLUDE_PRIVATE)
    add_custom_command(TARGET ${module}_stub POST_BUILD
      COMMAND ${Python_EXECUTABLE}
              ${CMAKE_CURRENT_SOURCE_DIR}/scripts/prepend_pyi_header.py ${_pyi})
  endfunction()

  valhalla_python_stub(_valhalla valhalla/_valhalla.pyi)
endif()

set(py_srcs
  valhalla/__init__.py
  valhalla/__version__.py
  valhalla/config.py
  valhalla/actor.py
  valhalla/baldr/__init__.py
  valhalla/baldr/utils/__init__.py
  valhalla/baldr/utils/graph_utils.py
  valhalla/midgard/__init__.py
  valhalla/midgard/utils/__init__.py
  valhalla/utils/__init__.py)

# scikit-build-core does a different build than CMake
if (SKBUILD)
  # SKBUILD_PLATLIB_DIR: the platlib path inside the wheel (site-packages or dist-packages or so)
  set(VALHALLA_INSTALL_PYDIR "${SKBUILD_PLATLIB_DIR}")

  list(APPEND py_srcs
    valhalla/__main__.py
    valhalla/_scripts.py)

  # install the executables when built with scikit-build-core (not with CMake directly)
  function(install_executable exe)
    install(TARGETS ${exe}
      DESTINATION "${VALHALLA_INSTALL_PYDIR}/valhalla/bin"
      COMPONENT python)
  endfunction()

  if (ENABLE_TOOLS)
    set(tools valhalla_service)
    foreach (exe ${tools})
      install_executable(${exe})
    endforeach()
  endif()

  if (ENABLE_DATA_TOOLS)
    set(data_tools valhalla_build_tiles valhalla_build_admins valhalla_add_predicted_traffic valhalla_add_landmarks valhalla_ingest_transit valhalla_convert_transit)
    foreach (exe ${data_tools})
      install_executable(${exe})
    endforeach()
  endif()
else()
  set(VALHALLA_INSTALL_PYDIR "${Python_SITEARCH}")
  # write the __version__.py
  file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/valhalla/__version__.py"
    "__version__ = \"${VERSION}\"\n")
endif()

# propagate up to the parent
set(Python_FOUND ${Python_FOUND} PARENT_SCOPE)

# copy valhalla python sources from src to build
function(set_output_python rel_path)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${rel_path} ${CMAKE_CURRENT_BINARY_DIR}/${rel_path} COPYONLY)
endfunction()

configure_file(${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_config ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla_build_config.py COPYONLY)
configure_file(${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_elevation ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla_build_elevation.py COPYONLY)
configure_file(${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_extract ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla_build_extract.py COPYONLY)

foreach (py_src ${py_srcs})
  set_output_python(${py_src})
endforeach()

install(TARGETS _valhalla
  DESTINATION "${VALHALLA_INSTALL_PYDIR}/valhalla"
  COMPONENT python)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/valhalla/_valhalla.pyi
  DESTINATION "${VALHALLA_INSTALL_PYDIR}/valhalla"
  COMPONENT python)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla
  DESTINATION "${VALHALLA_INSTALL_PYDIR}"
  COMPONENT python
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "__pycache__" EXCLUDE)
