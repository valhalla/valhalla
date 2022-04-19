# Install script for directory: /Users/cpark/Desktop/valhalla/src/meili

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla/meili" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/valhalla/meili/candidate_search.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/config.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/emission_cost_model.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/geometry_helpers.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/grid_range_query.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/grid_traversal.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/map_matcher.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/map_matcher_factory.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/match_result.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/measurement.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/priority_queue.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/routing.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/state.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/stateid.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/topk_search.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/traffic_segment_matcher.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/transition_cost_model.h"
    "/Users/cpark/Desktop/valhalla/valhalla/meili/viterbi_search.h"
    )
endif()

