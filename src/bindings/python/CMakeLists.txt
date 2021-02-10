add_subdirectory(${CMAKE_SOURCE_DIR}/third_party/pybind11 ${CMAKE_BINARY_DIR}/third_party/pybind11)

pybind11_add_module(python_valhalla python.cc)
target_link_libraries(python_valhalla PUBLIC valhalla)
set_target_properties(python_valhalla PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla)
set_target_properties(valhalla PROPERTIES POSITION_INDEPENDENT_CODE ON)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py ${CMAKE_CURRENT_BINARY_DIR}/valhalla/__init__.py COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/utils.py ${CMAKE_CURRENT_BINARY_DIR}/valhalla/utils.py COPYONLY)

# Part of OpenCVDetectPython.cmake
set(_executable ${PYTHON_EXECUTABLE})
set(_version_major_minor "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
if(CMAKE_HOST_UNIX)
execute_process(COMMAND ${_executable} -c "from distutils.sysconfig import *; print(get_python_lib())"
  RESULT_VARIABLE _cvpy_process
  OUTPUT_VARIABLE _std_packages_path
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if("${_std_packages_path}" MATCHES "site-packages")
  set(_packages_path "python${_version_major_minor}/site-packages")
else() #debian based assumed, install to the dist-packages.
  set(_packages_path "python${_version_major_minor}/dist-packages")
endif()
if(EXISTS "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}/${${packages_path}}")
  set(_packages_path "lib${LIB_SUFFIX}/${_packages_path}")
else()
  set(_packages_path "lib/${_packages_path}")
endif()
elseif(CMAKE_HOST_WIN32)
get_filename_component(_path "${_executable}" PATH)
file(TO_CMAKE_PATH "${_path}" _path)
if(NOT EXISTS "${_path}/Lib/site-packages")
  unset(_path)
  get_filename_component(_path "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${_version_major_minor}\\InstallPath]" ABSOLUTE)
  if(NOT _path)
    get_filename_component(_path "[HKEY_CURRENT_USER\\SOFTWARE\\Python\\PythonCore\\${_version_major_minor}\\InstallPath]" ABSOLUTE)
  endif()
  file(TO_CMAKE_PATH "${_path}" _path)
endif()
set(_packages_path "${_path}/Lib/site-packages")
unset(_path)
endif()
message(STATUS "Installing python modules to ${CMAKE_INSTALL_PREFIX}/${_packages_path}")
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla
DESTINATION "${_packages_path}"
COMPONENT python_valhalla
FILES_MATCHING PATTERN "*.py")
install(TARGETS python_valhalla
DESTINATION "${_packages_path}/valhalla"
COMPONENT python_valhalla)
