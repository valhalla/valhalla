# Install script for directory: /Users/cpark/Desktop/valhalla/src/sif

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla/sif" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/valhalla/sif/autocost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/bicyclecost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/costconstants.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/costfactory.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/dynamiccost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/edgelabel.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/hierarchylimits.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/motorcyclecost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/motorscootercost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/nocost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/osrm_car_duration.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/pedestriancost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/recost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/transitcost.h"
    "/Users/cpark/Desktop/valhalla/valhalla/sif/truckcost.h"
    )
endif()

