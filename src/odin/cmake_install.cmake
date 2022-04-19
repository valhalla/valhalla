# Install script for directory: /Users/cpark/Desktop/valhalla/src/odin

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla/odin" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/valhalla/odin/directionsbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/enhancedtrippath.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/maneuver.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/maneuversbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/markup_formatter.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/narrative_builder_factory.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/narrative_dictionary.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/narrativebuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/sign.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/signs.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/transitrouteinfo.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/util.h"
    "/Users/cpark/Desktop/valhalla/valhalla/odin/worker.h"
    )
endif()

