###############################################################################
# CMake module to search for SQLite 3 library
#
# On success, the macro sets the following variables:
# SQLITE3_FOUND = if the library found
# SQLITE3_LIBRARY = full path to the library
# SQLITE3_LIBRARIES = full path to the library
# SSQLITE3_INCLUDE_DIR = where to find the library headers
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################

set(SQLITE3_INCLUDE_SEARCH_PATHS
  /usr/include
  /usr/local/include
  $ENV{LIB_DIR}/include
  $ENV{LIB_DIR}/include/sqlite
  $ENV{LIB_DIR}/include/sqlite3
  $ENV{ProgramFiles}/SQLite/*/include
  $ENV{ProgramFiles}/SQLite3/*/include
  $ENV{SystemDrive}/SQLite/*/include
  $ENV{SystemDrive}/SQLite3/*/include
  $ENV{SQLITE_ROOT}/include
  ${SQLITE_ROOT_DIR}/include
  $ENV{OSGEO4W_ROOT}/include)


set(SQLITE3_LIB_SEARCH_PATHS
  /usr/lib
  /usr/local/lib
  $ENV{LIB_DIR}/lib
  $ENV{ProgramFiles}/SQLite/*/lib
  $ENV{ProgramFiles}/SQLite3/*/lib
  $ENV{SystemDrive}/SQLite/*/lib
  $ENV{SystemDrive}/SQLite3/*/lib
  $ENV{SQLITE_ROOT}/lib
  ${SQLITE_ROOT_DIR}/lib
  $ENV{OSGEO4W_ROOT}/lib)

if (APPLE)
  # on macOS, we try hard to prefer aftermarket sqlite3 installations, as the system sqlite3
  # disables extension loading, which breaks use of spatialite later on
  list(INSERT SQLITE3_INCLUDE_SEARCH_PATHS 0 /usr/local/opt/sqlite3/include /usr/local/include)
  find_path(SQLITE3_INCLUDE_DIR
    NAMES sqlite3.h
    PATHS ${SQLITE3_INCLUDE_SEARCH_PATHS}
    NO_DEFAULT_PATH)
else()
  find_path(SQLITE3_INCLUDE_DIR
    NAMES sqlite3.h
    PATHS ${SQLITE3_INCLUDE_SEARCH_PATHS})
endif()

IF (NOT SQLITE3_INCLUDE_DIR)
     MESSAGE(FATAL_ERROR "Could not find sqlite3.h in ${SQLITE3_INCLUDE_SEARCH_PATHS}")
ENDIF (NOT SQLITE3_INCLUDE_DIR)

set(SQLITE3_NAMES sqlite3_i sqlite3)

if (APPLE)
  # on macOS, we try hard to prefer aftermarket sqlite3 installations, as the system sqlite3
  # disables extension loading, which breaks use of spatialite later on
  list(INSERT SQLITE3_LIB_SEARCH_PATHS 0 /usr/local/opt/sqlite3/lib /usr/local/lib)
  find_library(SQLITE3_LIBRARY
    NAMES ${SQLITE3_NAMES}
    PATHS ${SQLITE3_LIB_SEARCH_PATHS}
    NO_DEFAULT_PATH)
else()
  find_library(SQLITE3_LIBRARY
    NAMES ${SQLITE3_NAMES}
    PATHS ${SQLITE3_LIB_SEARCH_PATHS})
endif()

set(SQLITE3_LIBRARIES
  ${SQLITE3_LIBRARIES}
  ${SQLITE3_LIBRARY})

# Handle the QUIETLY and REQUIRED arguments and set SQLITE3_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SQLITE3
  DEFAULT_MSG
  SQLITE3_LIBRARIES
  SQLITE3_INCLUDE_DIR)

mark_as_advanced(SQLITE3_LIBRARY SQLITE3_INCLUDE_DIR SQLITE3_LIBRARIES)

if(NOT SQLite3_FIND_QUIETLY)
  message(STATUS "Found SQLite3: ${SQLITE3_LIBRARY}")
endif(NOT SQLite3_FIND_QUIETLY)

check_library_exists("${SQLITE3_LIBRARY}" sqlite3_enable_load_extension "" SQLITE3_LOAD_EXTENSION)
IF (NOT SQLITE3_LOAD_EXTENSION)
    if (APPLE)
        MESSAGE(FATAL_ERROR "Found sqlite, but the sqlite3_enable_load_extension is disabled.  On macOS, you should `brew install sqlite3` as the system sqlite3 library disables extension loading by default for security reasons.")
    else()
        MESSAGE(FATAL_ERROR "Found sqlite, but the sqlite3_enable_load_extension is disabled")
    endif()
ENDIF (NOT SQLITE3_LOAD_EXTENSION)

add_library(SQLite3::SQLite3 INTERFACE IMPORTED)
set_target_properties(SQLite3::SQLite3 PROPERTIES
  INTERFACE_LINK_LIBRARIES "${SQLITE3_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${SQLITE3_INCLUDE_DIR}")
