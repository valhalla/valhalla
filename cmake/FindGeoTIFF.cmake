###############################################################################
#
# CMake module to search for GeoTIFF library
#
# On success, the macro sets the following variables:
# GEOTIFF_FOUND       = if the library found
# GEOTIFF_LIBRARIES   = full path to the library
# GEOTIFF_INCLUDE_DIR = where to find the library headers also defined,
#                       but not for general use are
# GEOTIFF_LIBRARY     = where to find the PROJ.4 library.
# GEOTIFF_VERSION     = version of library which was found, e.g. "1.2.5"
#
# Copyright (c) 2009 Mateusz Loskot <mateusz@loskot.net>
#
# Module source: http://github.com/mloskot/workshop/tree/master/cmake/
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#
###############################################################################
IF(GEOTIFF_INCLUDE_DIR)
  # Already in cache, be silent
  SET(GEOTIFF_FIND_QUIETLY TRUE)
ENDIF()

IF(WIN32)
  SET(OSGEO4W_IMPORT_LIBRARY geotiff_i)
  IF(DEFINED ENV{OSGEO4W_ROOT})
    SET(OSGEO4W_ROOT_DIR $ENV{OSGEO4W_ROOT})
    #MESSAGE(STATUS " FindGeoTIFF: trying OSGeo4W using environment variable OSGEO4W_ROOT=$ENV{OSGEO4W_ROOT}")
  ELSE()
    SET(OSGEO4W_ROOT_DIR c:/OSGeo4W)
    #MESSAGE(STATUS " FindGeoTIFF: trying OSGeo4W using default location OSGEO4W_ROOT=${OSGEO4W_ROOT_DIR}")
  ENDIF()
ENDIF()
     
FIND_PATH(GEOTIFF_INCLUDE_DIR
  geotiff.h
  PATH_SUFFIXES geotiff libgeotiff
  PATHS
  ${OSGEO4W_ROOT_DIR}/include)

SET(GEOTIFF_NAMES ${OSGEO4W_IMPORT_LIBRARY} geotiff)

FIND_LIBRARY(GEOTIFF_LIBRARY
  NAMES ${GEOTIFF_NAMES}
  PATHS
  ${OSGEO4W_ROOT_DIR}/lib)

IF(GEOTIFF_FOUND)
  SET(GEOTIFF_LIBRARIES ${GEOTIFF_LIBRARY})
ENDIF()

IF(GEOTIFF_INCLUDE_DIR)
  SET(GEOTIFF_VERSION 0)

  SET(GEOTIFF_VERSION_H "${GEOTIFF_INCLUDE_DIR}/geotiff.h")
  FILE(READ ${GEOTIFF_VERSION_H} GEOTIFF_VERSION_H_CONTENTS)

  IF (DEFINED GEOTIFF_VERSION_H_CONTENTS)
    STRING(REGEX REPLACE ".*#define[ \t]LIBGEOTIFF_VERSION[ \t]+([0-9]+).*" "\\1" GEOTIFF_VERSION_NUM "${GEOTIFF_VERSION_H_CONTENTS}")

    IF(NOT ${GEOTIFF_VERSION_NULL} MATCHES "[0-9]+")
      MESSAGE(FATAL_ERROR "GeoTIFF version parsing failed!")
    ENDIF()

    IF(GEOTIFF_VERSION_NUM AND NOT "${GEOTIFF_VERSION_NUM}" STREQUAL "0")
      MATH(EXPR GTIFF_VERSION_MAJOR "${GEOTIFF_VERSION_NUM} / 1000")
      MATH(EXPR GTIFF_VERSION_MINOR "${GEOTIFF_VERSION_NUM} % 1000 / 100")
      MATH(EXPR GTIFF_VERSION_PATCH "${GEOTIFF_VERSION_NUM} % 100 / 10")
    ENDIF()

    SET(GEOTIFF_VERSION "${GTIFF_VERSION_MAJOR}.${GTIFF_VERSION_MINOR}.${GTIFF_VERSION_PATCH}"
      CACHE INTERNAL "The version string for GeoTIFF library")

    IF (GEOTIFF_VERSION VERSION_EQUAL GeoTIFF_FIND_VERSION OR
        GEOTIFF_VERSION VERSION_GREATER GeoTIFF_FIND_VERSION)
      MESSAGE(STATUS "Found GeoTIFF version: ${GEOTIFF_VERSION}")
    ELSE()
      MESSAGE(FATAL_ERROR "GeoTIFF version check failed. Version ${GEOTIFF_VERSION} was found, at least version ${GeoTIFF_FIND_VERSION} is required")
    ENDIF()
  ELSE()
    MESSAGE(FATAL_ERROR "Failed to open ${GEOTIFF_VERSION_H} file")
  ENDIF()

ENDIF()

# Handle the QUIETLY and REQUIRED arguments and set GEOTIFF_FOUND to TRUE
# if all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GeoTIFF DEFAULT_MSG GEOTIFF_LIBRARY GEOTIFF_INCLUDE_DIR)

# export cmake target
if (GEOTIFF_FOUND AND NOT TARGET GeoTIFF::GeoTIFF)
  add_library(geotiff_library INTERFACE IMPORTED)
  set_property(TARGET geotiff_library PROPERTY
    INTERFACE_INCLUDE_DIRECTORIES "${GEOTIFF_INCLUDE_DIR}")
  set_property(TARGET geotiff_library PROPERTY
    INTERFACE_LINK_LIBRARIES
      "${GEOTIFF_LIBRARY}")
endif()