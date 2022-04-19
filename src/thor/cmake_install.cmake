# Install script for directory: /Users/cpark/Desktop/valhalla/src/thor

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Library/Developer/CommandLineTools/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla/thor" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/valhalla/thor/alternates.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/astar_bss.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/astarheuristic.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/bidirectional_astar.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/centroid.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/costmatrix.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/dijkstras.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/edgestatus.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/isochrone.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/map_matcher.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/matrix_common.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/multimodal.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/optimizer.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/pathalgorithm.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/pathinfo.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/route_matcher.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/timedistancebssmatrix.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/timedistancematrix.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/triplegbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/unidirectional_astar.h"
    "/Users/cpark/Desktop/valhalla/valhalla/thor/worker.h"
    )
endif()

