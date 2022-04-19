# Install script for directory: /Users/cpark/Desktop/valhalla/src

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/cpark/Desktop/valhalla/src/libvalhalla.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvalhalla.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvalhalla.a")
    execute_process(COMMAND "/Library/Developer/CommandLineTools/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvalhalla.a")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/doc/libvalhalla-dev" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/COPYING"
    "/Users/cpark/Desktop/valhalla/CHANGELOG.md"
    "/Users/cpark/Desktop/valhalla/README.md"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/valhalla" TYPE FILE FILES
    "/Users/cpark/Desktop/valhalla/valhalla/valhalla.h"
    "/Users/cpark/Desktop/valhalla/valhalla/worker.h"
    "/Users/cpark/Desktop/valhalla/valhalla/filesystem.h"
    "/Users/cpark/Desktop/valhalla/valhalla/proto_conversions.h"
    "/Users/cpark/Desktop/valhalla/valhalla/tile_server.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND /usr/local/Cellar/cmake/3.23.1/bin/cmake --build . --target libvalhalla.pc OUTPUT_QUIET ERROR_VARIABLE _err RESULT_VARIABLE _res)
  if(NOT ${_res} EQUAL 0)
    message(FATAL_ERROR "Configuring libvalhalla.pc failed: ${_err}")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevelopmentx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/Users/cpark/Desktop/valhalla/src/libvalhalla.pc")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/cpark/Desktop/valhalla/third_party/robin-hood-hashing/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/third_party/cpp-statsd-client/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/valhalla/proto/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/baldr/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/loki/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/meili/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/midgard/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/odin/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/sif/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/skadi/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/thor/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/tyr/cmake_install.cmake")
  include("/Users/cpark/Desktop/valhalla/src/mjolnir/cmake_install.cmake")

endif()

