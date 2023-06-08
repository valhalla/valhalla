include(GNUInstallDirs)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

## Set relative to the current source and binary paths
set(VALHALLA_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
if(NOT VALHALLA_SOURCE_DIR)
  set(VALHALLA_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
endif()

## Get Valhalla version
file(STRINGS "${VALHALLA_SOURCE_DIR}/valhalla/valhalla.h" version_lines REGEX "VALHALLA_VERSION_(MAJOR|MINOR|PATCH)")
foreach(line ${version_lines})
  if("${line}" MATCHES "(VALHALLA_VERSION_(MAJOR|MINOR|PATCH))[\t ]+([0-9]+)")
    set(${CMAKE_MATCH_1} ${CMAKE_MATCH_3})
    set(${CMAKE_MATCH_1} ${CMAKE_MATCH_3} PARENT_SCOPE)
  endif()
endforeach()
if(DEFINED VALHALLA_VERSION_MAJOR)
  set(VERSION "${VALHALLA_VERSION_MAJOR}")
  if(DEFINED VALHALLA_VERSION_MINOR)
    set(VERSION "${VERSION}.${VALHALLA_VERSION_MINOR}")
    if(DEFINED VALHALLA_VERSION_PATCH)
      set(VERSION "${VERSION}.${VALHALLA_VERSION_PATCH}")
    endif()
  endif()
else()
  message(FATAL_ERROR "No Valhalla major version")
endif()

## Declare C++ build configuration variables as part of HandleLibcxxFlags.
#
# - LIBCXX_COMPILE_FLAGS: flags used to compile libc++
# - LIBCXX_LINK_FLAGS: flags used to link libc++
# - LIBCXX_LIBRARIES: libraries to link libc++ to
set(LIBCXX_COMPILE_FLAGS "")
set(LIBCXX_LINK_FLAGS "")
set(LIBCXX_LIBRARIES "")

# Inlclude build macros for updating configuration variables
include(HandleLibcxxFlags)

## Define libvalhalla sub-modules as OBJECT libraries

# TODO: Remove workaround after switching to CMake 3.11
# with fixed  https://gitlab.kitware.com/cmake/cmake/issues/14778
# and merged https://gitlab.kitware.com/cmake/cmake/merge_requests/1524
# Check the commit message for the fix patch that removes cache internal variables
# and adds target_link_libraries(${library} ${MODULE_DEPENDS}) again
set(libvalhalla_compile_definitions "" CACHE INTERNAL "")
set(libvalhalla_compile_options "" CACHE INTERNAL "")
set(libvalhalla_include_directories "" CACHE INTERNAL "")
set(libvalhalla_link_objects "" CACHE INTERNAL  "")
set(libvalhalla_link_libraries "" CACHE INTERNAL  "")

# Setup warning flags. Note that these are only added to libvalhalla_compile_options if
# ENABLE_COMPILER_WARNINGS=ON. Note that this needs to be done before the definition of
# valhalla_module, otherwise LIBCXX_COMPILE_FLAGS will not available for downstream targets.

if (ENABLE_COMPILER_WARNINGS)
  message(STATUS "Enabling compiler warning flags")
endif()

function (cxx_add_warning_flags target)
  target_add_compile_flags_if_supported(${target} PRIVATE -Wall -Wextra)
  if (ENABLE_WERROR)
   target_add_compile_flags_if_supported(${target} PRIVATE -Werror)
  endif()
endfunction()

function(valhalla_module)
  set(oneValueArgs NAME)
  set(multiValueArgs SOURCES SOURCES_WITH_WARNINGS HEADERS DEPENDS INCLUDE_DIRECTORIES SYSTEM_INCLUDE_DIRECTORIES)
  cmake_parse_arguments(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if (UNIX AND ENABLE_SINGLE_FILES_WERROR)
    set_source_files_properties(
            ${MODULE_SOURCES}
            PROPERTIES
            COMPILE_FLAGS
            "-Werror -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers")
  endif()
  list(APPEND MODULE_SOURCES ${MODULE_SOURCES_WITH_WARNINGS})

  set(library valhalla-${MODULE_NAME})
  if (ENABLE_STATIC_LIBRARY_MODULES)
    add_library(${library} STATIC ${MODULE_SOURCES} ${MODULE_HEADERS})
  else()
    add_library(${library} OBJECT ${MODULE_SOURCES} ${MODULE_HEADERS})
  endif()
  add_library(valhalla::${MODULE_NAME} ALIAS ${library})
  set_target_properties(${library} PROPERTIES FOLDER "Modules")

  # Generate source groups so the files are properly sorted in IDEs like Xcode.
  create_source_groups("Header Files" ${MODULE_HEADERS})
  create_source_groups("Source Files" ${MODULE_SOURCES})

  target_compile_definitions(${library}
    PUBLIC
      $<$<BOOL:${WIN32}>:NOGDI;WIN32_LEAN_AND_MEAN>
      $<$<BOOL:${MSVC}>:NOMINMAX>
      HAS_REMOTE_API=0
      AUTO_DOWNLOAD=0
    PRIVATE
      $<$<BOOL:${LOGGING_LEVEL}>:LOGGING_LEVEL_${LOGGING_LEVEL}>)
  target_include_directories(${library} ${MODULE_INCLUDE_DIRECTORIES}
     PRIVATE
       ${VALHALLA_BINARY_DIR}
       ${VALHALLA_BINARY_DIR}/valhalla)
  if (MODULE_SYSTEM_INCLUDE_DIRECTORIES)
    target_include_directories(${library} SYSTEM ${MODULE_SYSTEM_INCLUDE_DIRECTORIES})
  endif()

  target_link_libraries(${library} Boost::boost)

  if(ENABLE_COVERAGE)
    target_compile_options(${library} PUBLIC -ftest-coverage -fprofile-arcs)
  endif()

  # On more modern x86 processors that support SSE2, specifying the compiler options -mfpmath=sse -msse2
  # ensures all float and double operations are performed in SSE registers and correctly rounded.
  # These options do not affect the ABI and should therefore be used whenever possible for
  # predictable numerical results
  # For the x86-64 compiler, these extensions are enabled by default.
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "amd64|AMD64|x64|x86|x86_64" AND NOT CMAKE_OSX_ARCHITECTURES MATCHES "arm64")
    target_add_compile_flags_if_supported(${library} PRIVATE -mfpmath=sse -msse2)
  endif()

  if(ENABLE_COMPILER_WARNINGS)
    cxx_add_warning_flags(${library})
  endif()

  install(FILES ${MODULE_HEADERS}
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${MODULE_NAME}"
    COMPONENT development)

  if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
    set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif()

  ## Save properties of external dependencies and target objects to propagate to valhalla target
  string(REGEX REPLACE "(valhalla::[^;]+)" "" external_module_depends "${MODULE_DEPENDS}")
  string(REGEX REPLACE "([^;]+)" "$<TARGET_PROPERTY:\\1,INTERFACE_COMPILE_DEFINITIONS>" external_compile_definitions "${external_module_depends}")
  string(REGEX REPLACE "([^;]+)" "$<TARGET_PROPERTY:\\1,INTERFACE_INCLUDE_DIRECTORIES>" external_include_directories "${external_module_depends}")
  target_compile_definitions(${library} PRIVATE ${external_compile_definitions})
  target_include_directories(${library} PRIVATE ${external_include_directories})
  foreach(dep "${MODULE_DEPENDS}")
    if("${dep}" MATCHES "^valhalla::")
      add_dependencies(${library} ${dep})
    endif()
  endforeach()

  set(libvalhalla_compile_definitions "${libvalhalla_compile_definitions};$<TARGET_PROPERTY:${library},COMPILE_DEFINITIONS>" CACHE INTERNAL "")
  set(libvalhalla_compile_options "${libvalhalla_compile_options};$<TARGET_PROPERTY:${library},COMPILE_OPTIONS>" CACHE INTERNAL "")
  set(libvalhalla_include_directories "${libvalhalla_include_directories};$<TARGET_PROPERTY:${library},INCLUDE_DIRECTORIES>" CACHE INTERNAL "")
  if (ENABLE_STATIC_LIBRARY_MODULES)
    set(libvalhalla_link_libraries "${libvalhalla_link_libraries};${external_module_depends};${library}" CACHE INTERNAL "")
  else()
    set(libvalhalla_link_objects "${libvalhalla_link_objects};$<TARGET_OBJECTS:${library}>" CACHE INTERNAL "")
    set(libvalhalla_link_libraries "${libvalhalla_link_libraries};${external_module_depends}" CACHE INTERNAL "")
  endif()
endfunction()

add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/robin-hood-hashing ${CMAKE_BINARY_DIR}/third_party/robin-hood-hashing)
SET(CPP_STATSD_STANDALONE OFF CACHE BOOL "ignore test targets" FORCE)
add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/cpp-statsd-client ${CMAKE_BINARY_DIR}/third_party/cpp-statsd-client)

add_subdirectory(../proto ${CMAKE_CURRENT_BINARY_DIR}/valhalla/proto)
add_subdirectory(baldr)
add_subdirectory(loki)
add_subdirectory(meili)
add_subdirectory(midgard)
add_subdirectory(odin)
add_subdirectory(sif)
add_subdirectory(skadi)
add_subdirectory(thor)
add_subdirectory(tyr)
if(ENABLE_DATA_TOOLS)
  add_subdirectory(mjolnir)
endif()

## libvalhalla

set(valhalla_hdrs
    ${VALHALLA_SOURCE_DIR}/valhalla/valhalla.h
    ${VALHALLA_SOURCE_DIR}/valhalla/worker.h
    ${VALHALLA_SOURCE_DIR}/valhalla/filesystem.h
    ${VALHALLA_SOURCE_DIR}/valhalla/proto_conversions.h
    )

set(valhalla_src
    worker.cc
    filesystem.cc
    proto_conversions.cc
    ${VALHALLA_SOURCE_DIR}/valhalla/config.h
    ${valhalla_hdrs}
    ${libvalhalla_link_objects})

if (UNIX AND ENABLE_SINGLE_FILES_WERROR)
  # Enables stricter compiler checks on a file-by-file basis
  # which allows us to migrate piecemeal
  set_source_files_properties(proto_conversions.cc PROPERTIES COMPILE_FLAGS "-Wall -Werror")
endif()

if (ENABLE_SERVICES)
  set(valhalla_hdrs ${valhalla_hdrs} ${VALHALLA_SOURCE_DIR}/valhalla/tile_server.h)
  set(valhalla_src ${valhalla_src} tile_server.cc)
endif()

add_library(valhalla ${valhalla_src})
set_target_properties(valhalla PROPERTIES FOLDER "Library")
create_source_groups("Source Files" ${valhalla_src})

target_compile_definitions(valhalla
  PUBLIC
    $<$<BOOL:${WIN32}>:NOGDI;WIN32_LEAN_AND_MEAN>
    $<$<BOOL:${MSVC}>:NOMINMAX>
    HAS_REMOTE_API=0
    AUTO_DOWNLOAD=0
  PRIVATE
    ${libvalhalla_compile_definitions})

target_compile_options(valhalla PUBLIC ${libvalhalla_compile_options})

if(ENABLE_COMPILER_WARNINGS)
  cxx_add_warning_flags(valhalla)
endif()

target_include_directories(valhalla
  PUBLIC
    ${VALHALLA_SOURCE_DIR}
    ${VALHALLA_SOURCE_DIR}/valhalla # TODO: this path must be removed and changed to #include <valhalla/...> in headers
    ${VALHALLA_BINARY_DIR}
    ${VALHALLA_BINARY_DIR}/valhalla  # TODO: this path must be removed and changed to #include <valhalla/...> in headers
    ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
    ${VALHALLA_SOURCE_DIR}/third_party/date/include
    ${VALHALLA_SOURCE_DIR}/third_party/cpp-statsd-client/include
    $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
  PRIVATE
    ${libvalhalla_include_directories})

target_link_libraries(valhalla
  PUBLIC
    ${libvalhalla_link_libraries}
  PRIVATE
    $<$<BOOL:${ENABLE_COVERAGE}>:gcov>
    Threads::Threads
    cpp-statsd-client)

set_target_properties(valhalla PROPERTIES
  VERSION "${VERSION}"
  SOVERSION "${VALHALLA_VERSION_MAJOR}")


install(TARGETS valhalla
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shared NAMELINK_SKIP
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT development)

if(BUILD_SHARED_LIBS)
  install(FILES ${VALHALLA_SOURCE_DIR}/COPYING ${VALHALLA_SOURCE_DIR}/CHANGELOG.md
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/libvalhalla0"
    COMPONENT shared)

  set(LIBVALHALLA_SO_LINK ${CMAKE_SHARED_LIBRARY_PREFIX}valhalla${CMAKE_SHARED_LIBRARY_SUFFIX})

  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${LIBVALHALLA_SO_LINK}
    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    COMPONENT development)

  if(UNIX)
    add_custom_target(${LIBVALHALLA_SO_LINK} ALL
      COMMAND ${CMAKE_COMMAND} -E create_symlink "${LIBVALHALLA_SO_LINK}.${VALHALLA_VERSION_MAJOR}" ${LIBVALHALLA_SO_LINK})
    set_target_properties(${LIBVALHALLA_SO_LINK} PROPERTIES FOLDER "Library")
  endif()
endif()

install(FILES ${VALHALLA_SOURCE_DIR}/COPYING ${VALHALLA_SOURCE_DIR}/CHANGELOG.md ${VALHALLA_SOURCE_DIR}/README.md
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/libvalhalla-dev"
  COMPONENT development)

install(FILES ${valhalla_hdrs}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/valhalla"
  COMPONENT development)

if(PKG_CONFIG_FOUND)
  ## Configure libvalhalla.pc file with valhalla target linking options via PkgConfig linker
  set(CMAKE_PkgConfig_LINK_EXECUTABLE "<CMAKE_COMMAND> -DINPUT=${VALHALLA_SOURCE_DIR}/libvalhalla.pc.in -DOUTPUT=<TARGET> -DVERSION=${VERSION} -Dprefix=${CMAKE_INSTALL_PREFIX} -Dexec_prefix=${CMAKE_INSTALL_PREFIX} -Dlibdir=${CMAKE_INSTALL_LIBDIR} -Dincludedir=${CMAKE_INSTALL_INCLUDEDIR} -Ddeplibs=\"<FLAGS> <LINK_FLAGS> <LINK_LIBRARIES>\" -P ${CMAKE_SOURCE_DIR}/cmake/PkgConfig.cmake")

  add_executable(libvalhalla.pc EXCLUDE_FROM_ALL ../libvalhalla.pc.in)
  target_link_libraries(libvalhalla.pc valhalla)
  set_target_properties(libvalhalla.pc PROPERTIES LINKER_LANGUAGE PkgConfig)
  set_target_properties(libvalhalla.pc PROPERTIES FOLDER "Library")
  install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} --build . --target libvalhalla.pc OUTPUT_QUIET ERROR_VARIABLE _err RESULT_VARIABLE _res)
  if(NOT \${_res} EQUAL 0)
    message(FATAL_ERROR \"Configuring libvalhalla.pc failed: \${_err}\")
  endif()")
  install(FILES ${VALHALLA_BINARY_DIR}/libvalhalla.pc
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
    COMPONENT development)
endif()
