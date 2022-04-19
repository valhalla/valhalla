# Install script for directory: /Users/cpark/Desktop/valhalla/src/mjolnir

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla/mjolnir" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/admin.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/adminbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/adminconstants.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/bssbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/complexrestrictionbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/countryaccess.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/dataquality.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/directededgebuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/edgeinfobuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/elevationbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/ferry_connections.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/graphbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/graphenhancer.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/graphfilter.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/graphtilebuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/graphvalidator.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/hierarchybuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/linkclassification.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/luatagtransform.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/node_expander.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmaccess.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmaccessrestriction.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmadmin.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmadmindata.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmdata.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmnode.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmpbfparser.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmpronunciation.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmrestriction.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/osmway.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/pbfadminparser.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/pbfgraphparser.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/restrictionbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/servicedays.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/shortcutbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/timeparsing.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/transitbuilder.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/transitpbf.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/uniquenames.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/util.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/valhalla_add_elevation_utils.h"
    "/Users/cpark/Desktop/valhalla/valhalla/mjolnir/validatetransit.h"
    )
endif()

