if (PREFER_EXTERNAL_DEPS)
  find_package(pybind11 QUIET)
  if (NOT pybind11_FOUND)
    message(WARNING "No pybind11 found in system libraries, using vendored pybind11...")
  endif()
endif()

if (NOT TARGET pybind11::pybind11_headers)
  add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/pybind11 ${CMAKE_BINARY_DIR}/third_party/pybind11)
endif()

message(STATUS "Installing python modules to ${Python_SITEARCH}")

function(set_output_python rel_path)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${rel_path} ${CMAKE_CURRENT_BINARY_DIR}/${rel_path} COPYONLY)
endfunction()

# add and install module(s)
pybind11_add_module(_valhalla ${CMAKE_CURRENT_SOURCE_DIR}/src/_valhalla.cc)
target_link_libraries(_valhalla PRIVATE valhalla $<$<BOOL:${ENABLE_COVERAGE}>:gcov>)
pybind11_add_module(graph_utils 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_utils.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/graph_id.cc)
target_link_libraries(graph_utils PRIVATE valhalla $<$<BOOL:${ENABLE_COVERAGE}>:gcov>)

set_target_properties(_valhalla PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla)
set_target_properties(graph_utils PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla/utils)
set_target_properties(valhalla PROPERTIES POSITION_INDEPENDENT_CODE ON)

# the package we're building, pyvalhalla or pyvalhalla-weekly
if(NOT VALHALLA_PYTHON_PACKAGE)
  set(VALHALLA_PYTHON_PACKAGE "pyvalhalla-weekly")
endif()
target_compile_definitions(_valhalla PRIVATE -DVALHALLA_PYTHON_PACKAGE=${VALHALLA_PYTHON_PACKAGE})

# copy valhalla python sources from src to build
set(py_srcs 
  valhalla/__init__.py
  valhalla/config.py
  valhalla/actor.py
  valhalla/utils/__init__.py
  valhalla/utils/graph_utils.pyi
  valhalla/utils/decode_polyline.py)
foreach (py_src ${py_srcs})
  set_output_python(${py_src})
endforeach()
configure_file(${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_config ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla_build_config.py COPYONLY)

install(TARGETS _valhalla
  DESTINATION "${Python_SITEARCH}/valhalla"
  COMPONENT runtime)
install(TARGETS graph_utils
  DESTINATION "${Python_SITEARCH}/valhalla/utils"
  COMPONENT runtime)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla ${CMAKE_CURRENT_BINARY_DIR}/valhalla/utils
  DESTINATION "${Python_SITEARCH}"
  COMPONENT runtime
  FILES_MATCHING PATTERN "*.py*")
