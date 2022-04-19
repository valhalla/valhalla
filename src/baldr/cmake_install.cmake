# Install script for directory: /Users/cpark/Desktop/valhalla/src/baldr

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla/baldr" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/accessrestriction.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/admin.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/admininfo.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/attributes_controller.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/complexrestriction.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/compression_utils.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/connectivity_map.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/curl_tilegetter.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/curler.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/datetime.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/directededge.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/double_bucket_queue.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/edgeinfo.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/edgetracker.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/graphconstants.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/graphid.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/graphmemory.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/graphreader.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/graphtile.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/graphtileheader.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/graphtileptr.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/json.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/laneconnectivity.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/location.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/merge.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/nodeinfo.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/nodetransition.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/openlr.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/pathlocation.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/predictedspeeds.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/rapidjson_utils.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/sign.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/signinfo.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/streetname.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/streetname_us.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/streetnames.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/streetnames_factory.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/streetnames_us.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/tilegetter.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/tilehierarchy.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/time_info.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/timedomain.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/traffictile.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/transitdeparture.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/transitroute.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/transitschedule.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/transitstop.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/transittransfer.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/turn.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/turnlanes.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/verbal_text_formatter.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/verbal_text_formatter_factory.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/verbal_text_formatter_us.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/verbal_text_formatter_us_co.h"
    "/Users/cpark/Desktop/valhalla/valhalla/baldr/verbal_text_formatter_us_tx.h"
    )
endif()

