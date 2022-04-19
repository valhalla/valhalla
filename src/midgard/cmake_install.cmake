# Install script for directory: /Users/cpark/Desktop/valhalla/src/midgard

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla/midgard" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/aabb2.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/constants.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/distanceapproximator.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/ellipse.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/encoded.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/gridded_data.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/linesegment2.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/logging.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/obb2.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/point2.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/point_tile_index.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/pointll.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/polyline2.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/sequence.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/tiles.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/util.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/util_core.h"
    "/Users/cpark/Desktop/valhalla/valhalla/midgard/vector2.h"
    )
endif()

