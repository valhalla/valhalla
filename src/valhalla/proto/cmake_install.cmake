# Install script for directory: /Users/cpark/Desktop/valhalla/proto

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla/proto" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/api.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/common.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/directions.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/info.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/options.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/sign.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/trip.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/transit.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/transit_fetch.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/incidents.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/status.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/fileformat.pb.h"
    "/Users/cpark/Desktop/valhalla/src/valhalla/proto/osmformat.pb.h"
    )
endif()

