# Finds liblz4.
#
# This module defines:
# LZ4_FOUND
# LZ4_INCLUDE_DIR
# LZ4_LIBRARY
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
find_path(LZ4_INCLUDE_DIR NAMES lz4.h)

if( LZ4_USE_STATIC_LIBS )
  set( _lz4_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
  if(WIN32)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .lib .a ${CMAKE_FIND_LIBRARY_SUFFIXES})
  else()
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a )
  endif()
endif()

find_library(LZ4_LIBRARY NAMES lz4)

if( LZ4_USE_STATIC_LIBS )
  set(CMAKE_FIND_LIBRARY_SUFFIXES ${_lz4_ORIG_CMAKE_FIND_LIBRARY_SUFFIXES})
endif()

# We require LZ4_compress_default() which was added in v1.7.0
if (LZ4_LIBRARY)
  include(CheckCSourceRuns)
  set(CMAKE_REQUIRED_INCLUDES ${LZ4_INCLUDE_DIR})
  set(CMAKE_REQUIRED_LIBRARIES ${LZ4_LIBRARY})
  check_c_source_runs("
#include <lz4.h>
int main() {
  int good = (LZ4_VERSION_MAJOR > 1) ||
    ((LZ4_VERSION_MAJOR == 1) && (LZ4_VERSION_MINOR >= 7));
return !good;
}" LZ4_GOOD_VERSION)
  set(CMAKE_REQUIRED_INCLUDES)
  set(CMAKE_REQUIRED_LIBRARIES)
endif()

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
    LZ4 DEFAULT_MSG
    LZ4_LIBRARY LZ4_INCLUDE_DIR LZ4_GOOD_VERSION)

mark_as_advanced(LZ4_INCLUDE_DIR LZ4_LIBRARY)

add_library(LZ4::LZ4 INTERFACE IMPORTED)
set_target_properties(LZ4::LZ4 PROPERTIES
  INTERFACE_LINK_LIBRARIES "${LZ4_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${LZ4_INCLUDE_DIR}")
