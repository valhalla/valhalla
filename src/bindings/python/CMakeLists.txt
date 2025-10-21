if (PREFER_EXTERNAL_DEPS)
  find_package(nanobind QUIET CONFIG)
  if (NOT TARGET nanobind::nanobind)
    message(WARNING "No nanobind found in system libraries, using vendored nanobind...")
  endif()
endif()

if (NOT TARGET nanobind::nanobind)
  add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/nanobind ${CMAKE_BINARY_DIR}/third_party/nanobind EXCLUDE_FROM_ALL)
endif()

# determine the package name and write it to pyproject.toml
if (NOT VALHALLA_RELEASE_PKG)
  set(VALHALLA_RELEASE_PKG "pyvalhalla-weekly")
endif()

# scikit-build-core does a different build than CMake
if (SKBUILD)
  # determine the build folder for the bindings
  # for cmake, this doesn't need to be set, it's only used in pyproject.toml.in
  if (NOT VALHALLA_BUILD_FOLDER)
    set(VALHALLA_BUILD_FOLDER ${CMAKE_CURRENT_BINARY_DIR})
  endif()

  # this will be set by CI/cibuildwheel for releases
  # for manual installations, this should be added to CMake config or it's empty
  if (NOT VALHALLA_VERSION_MODIFIER)
    target_compile_definitions(_valhalla PRIVATE VALHALLA_VERSION_MODIFIER="")
  endif()
  
  # SKBUILD_PLATLIB_DIR: the platlib path inside the wheel (site-packages or dist-packages or so)
  set(VALHALLA_INSTALL_PYDIR "${SKBUILD_PLATLIB_DIR}")

  # install the executables when built with scikit-build-core (not with CMake directly)
  list(APPEND py_srcs
    valhalla/__version__.py  # will already exist at this point by setuptools_scm
    valhalla/__main__.py
    valhalla/_scripts.py)

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

  configure_file(${VALHALLA_SOURCE_DIR}/pyproject.toml.in ${VALHALLA_SOURCE_DIR}/pyproject.toml @ONLY)
else()
  set(VALHALLA_INSTALL_PYDIR "${Python_SITEARCH}")
endif()

message(STATUS "Installing python modules to ${Python_SITEARCH}")

# add and install module(s)
nanobind_add_module(_valhalla 
  NB_STATIC
  STABLE_ABI
  LTO
  ${CMAKE_CURRENT_SOURCE_DIR}/src/_valhalla.cc)
target_link_libraries(_valhalla PRIVATE valhalla $<$<BOOL:${ENABLE_COVERAGE}>:gcov>)
nanobind_add_module(graph_utils
  NB_STATIC
  STABLE_ABI
  LTO
  ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_utils.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_id.cc)
target_link_libraries(graph_utils PRIVATE valhalla $<$<BOOL:${ENABLE_COVERAGE}>:gcov>)

if(APPLE)
  set_target_properties(_valhalla PROPERTIES INSTALL_RPATH "@loader_path")
  set_target_properties(graph_utils PROPERTIES INSTALL_RPATH "@loader_path")
elseif(UNIX AND NOT APPLE)
  set_target_properties(_valhalla PROPERTIES INSTALL_RPATH "\$ORIGIN")
  set_target_properties(graph_utils PROPERTIES INSTALL_RPATH "\$ORIGIN")
endif()

set_target_properties(_valhalla PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla)
set_target_properties(graph_utils PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla/utils)
set_target_properties(valhalla PROPERTIES POSITION_INDEPENDENT_CODE ON)

# the package we're building, pyvalhalla or pyvalhalla-weekly
if(NOT VALHALLA_PYTHON_PACKAGE)
  set(VALHALLA_PYTHON_PACKAGE "pyvalhalla-weekly")
endif()

target_compile_definitions(_valhalla PRIVATE -DVALHALLA_PYTHON_PACKAGE=${VALHALLA_PYTHON_PACKAGE})

set(py_srcs 
  valhalla/__init__.py
  valhalla/config.py
  valhalla/actor.py
  valhalla/utils/__init__.py
  valhalla/utils/graph_utils.pyi
  valhalla/utils/decode_polyline.py)

# copy valhalla python sources from src to build
function(set_output_python rel_path)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${rel_path} ${CMAKE_CURRENT_BINARY_DIR}/${rel_path} COPYONLY)
endfunction()
configure_file(${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_config ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla_build_config.py COPYONLY)

foreach (py_src ${py_srcs})
  set_output_python(${py_src})
endforeach()

install(TARGETS _valhalla
  DESTINATION "${VALHALLA_INSTALL_PYDIR}/valhalla"
  COMPONENT python)
install(TARGETS graph_utils
  DESTINATION "${VALHALLA_INSTALL_PYDIR}/valhalla/utils"
  COMPONENT python)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla
  DESTINATION "${VALHALLA_INSTALL_PYDIR}"
  COMPONENT python
  FILES_MATCHING    
    PATTERN "*.py*"
    PATTERN "__pycache__" EXCLUDE)
