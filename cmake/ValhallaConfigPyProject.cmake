# Usage:
#   cmake \
#     -DPYVALHALLA_NAME=pyvalhalla \
#     -P ValhallaConfigPyProject.cmake
#
# It will generate pyproject.toml in ${VALHALLA_PYTHON_BUILD_DIR}.

cmake_minimum_required(VERSION 3.15)

# the only template parameter for pyproject.toml.in's project.name
if(NOT DEFINED PYVALHALLA_NAME)
  message(STATUS "PYVALHALLA_NAME not set; defaulting to 'pyvalhalla-weekly'")
  set(PYVALHALLA_NAME "pyvalhalla-weekly")
endif()

set(_template "${CMAKE_SOURCE_DIR}/pyproject.toml.in")
set(_output   "${CMAKE_SOURCE_DIR}/pyproject.toml")

if(NOT EXISTS "${_template}")
  message(FATAL_ERROR "Template file not found: ${_template}")
endif()

message(STATUS "Configuring pyproject.toml:")
message(STATUS "  Template: ${_template}")
message(STATUS "  Output:   ${_output}")

configure_file(
  "${_template}"
  "${_output}"
  @ONLY
)

message(STATUS "pyproject.toml successfully configured")
