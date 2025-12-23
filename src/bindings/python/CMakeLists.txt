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

# add and install module(s)
nanobind_add_module(_valhalla 
  NB_STATIC
  STABLE_ABI
  LTO
  ${CMAKE_CURRENT_SOURCE_DIR}/src/_valhalla.cc)
nanobind_add_module(graph_utils
  NB_STATIC
  STABLE_ABI
  LTO
  ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_utils.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_id.cc)

foreach(tgt IN ITEMS _valhalla graph_utils)
  # force abi3.so extension
  nanobind_extension_abi3(${tgt})

  target_link_libraries(${tgt} PRIVATE 
    valhalla
    Python::SABIModule
    $<$<BOOL:${ENABLE_COVERAGE}>:gcov>)
endforeach()

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

set(py_srcs 
  valhalla/__init__.py
  valhalla/__version__.py
  valhalla/config.py
  valhalla/actor.py
  valhalla/utils/__init__.py
  valhalla/utils/graph_utils.pyi
  valhalla/utils/decode_polyline.py)

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
