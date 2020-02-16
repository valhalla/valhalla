# Find SpatiaLite
# ~~~~~~~~~~~~~~~
# Copyright (c) 2009, Sandro Furieri <a.furieri at lqt.it>
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
# CMake module to search for SpatiaLite library
#
# If it's found it sets SPATIALITE_FOUND to TRUE
# and following variables are set:
#    SPATIALITE_INCLUDE_DIR
#    SPATIALITE_LIBRARY

# This macro checks if the symbol exists
include(CheckLibraryExists)


# FIND_PATH and FIND_LIBRARY normally search standard locations
# before the specified paths. To search non-standard paths first,
# FIND_* is invoked first with specified paths and NO_DEFAULT_PATH
# and then again with no specified paths to search the default
# locations. When an earlier FIND_* succeeds, subsequent FIND_*s
# searching for the same item do nothing.

# try to use sqlite framework on mac
# want clean framework path, not unix compatibility path
IF (APPLE)
  IF (CMAKE_FIND_FRAMEWORK MATCHES "FIRST"
      OR CMAKE_FRAMEWORK_PATH MATCHES "ONLY"
      OR NOT CMAKE_FIND_FRAMEWORK)
    SET (CMAKE_FIND_FRAMEWORK_save ${CMAKE_FIND_FRAMEWORK} CACHE STRING "" FORCE)
    SET (CMAKE_FIND_FRAMEWORK "ONLY" CACHE STRING "" FORCE)
    FIND_PATH(SPATIALITE_INCLUDE_DIR SQLite3/spatialite.h)
    # if no SpatiaLite header, we don't want SQLite find below to succeed
    IF (SPATIALITE_INCLUDE_DIR)
      FIND_LIBRARY(SPATIALITE_LIBRARY SQLite3)
      # FIND_PATH doesn't add "Headers" for a framework
      SET (SPATIALITE_INCLUDE_DIR ${SPATIALITE_LIBRARY}/Headers CACHE PATH "Path to a file." FORCE)
    ENDIF (SPATIALITE_INCLUDE_DIR)
    SET (CMAKE_FIND_FRAMEWORK ${CMAKE_FIND_FRAMEWORK_save} CACHE STRING "" FORCE)
  ENDIF ()
ENDIF (APPLE)

FIND_PATH(SPATIALITE_INCLUDE_DIR spatialite.h
  /usr/include
  "$ENV{INCLUDE}"
  "$ENV{LIB_DIR}/include"
  "$ENV{LIB_DIR}/include/spatialite"
  )

FIND_LIBRARY(SPATIALITE_LIBRARY NAMES spatialite_i spatialite PATHS
  /usr/lib
  $ENV{LIB}
  $ENV{LIB_DIR}/lib
  )

# Handle the QUIETLY and REQUIRED arguments and set SQLITE3_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SpatiaLite
  DEFAULT_MSG
  SPATIALITE_LIBRARY
  SPATIALITE_INCLUDE_DIR)

mark_as_advanced(SPATIALITE_LIBRARY SPATIALITE_INCLUDE_DIR SPATIALITE_LIBRARY)

IF (SPATIALITE_FOUND)
  set(SPATIALITE_LIBRARIES ${SPATIALITE_LIBRARY})

   IF(APPLE)
     # no extra LDFLAGS used in link test, may fail in OS X SDK
     SET(CMAKE_REQUIRED_LIBRARIES "-F/Library/Frameworks" ${CMAKE_REQUIRED_LIBRARIES})
   ENDIF(APPLE)

   check_library_exists("${SPATIALITE_LIBRARY}" gaiaStatisticsInvalidate "" SPATIALITE_VERSION_GE_4_2_0)
   IF (NOT SPATIALITE_VERSION_GE_4_2_0)
     MESSAGE(FATAL_ERROR "Found SpatiaLite, but version is too old. Requires at least version 4.2.0")
   ENDIF (NOT SPATIALITE_VERSION_GE_4_2_0)

  if(NOT TARGET SpatiaLite::SpatiaLite)
    add_library(SpatiaLite::SpatiaLite INTERFACE IMPORTED)

    set_target_properties(SpatiaLite::SpatiaLite
      PROPERTIES
      INTERFACE_LINK_LIBRARIES "${SPATIALITE_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${SPATIALITE_INCLUDE_DIR}")
  endif()
ENDIF (SPATIALITE_FOUND)
