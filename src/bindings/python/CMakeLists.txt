find_package(PythonInterp)
find_package(PythonLibs)

## This a workaround for CMake Boost::python configuration
set(Boost_Python_libs
  python${PYTHON_VERSION_MAJOR}
  python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}
  python-py${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
if(NOT ${PYTHON_VERSION_MAJOR} EQUAL 3)
  list(INSERT Boost_Python_libs 0 "python")
endif()

foreach(lib ${Boost_Python_libs})
  set(Boost_Python_lib "${lib}")
  string(TOUPPER ${lib} PYTHON_LIB)
  set(_Boost_${PYTHON_LIB}_HEADERS "boost/python.hpp")
  find_package(Boost COMPONENTS ${lib} QUIET)
  if(${Boost_${PYTHON_LIB}_FOUND})
    message(STATUS "Found the following Boost libraries:")
    message(STATUS "  ${Boost_Python_lib}")
    break()
  endif()
endforeach()

if(NOT Boost_${PYTHON_LIB}_FOUND)
  message(SEND_ERROR "Boost::python not found")
else()
  python_add_module(python_valhalla python.cc)
  set_target_properties(python_valhalla PROPERTIES
    OUTPUT_NAME valhalla
    LINK_LIBRARIES "${PYTHON_LIBRARIES}"
    INCLUDE_DIRECTORIES "${PYTHON_INCLUDE_DIRS}")
  target_link_libraries(python_valhalla valhalla Boost::${Boost_Python_lib})
  set_target_properties(valhalla PROPERTIES POSITION_INDEPENDENT_CODE ON)

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
  install(TARGETS python_valhalla
    DESTINATION "${_packages_path}"
    COMPONENT python)
endif()
