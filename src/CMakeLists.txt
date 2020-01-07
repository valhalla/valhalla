include(GNUInstallDirs)

## Set relative to the current source and binary paths
set(VALHALLA_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
if(NOT VALHALLA_SOURCE_DIR)
  set(VALHALLA_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
endif()

## Get Valhalla version
file(STRINGS "${VALHALLA_SOURCE_DIR}/valhalla/valhalla.h.in" version_lines REGEX "VALHALLA_VERSION_(MAJOR|MINOR|PATCH)")
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

function(valhalla_module)
  set(oneValueArgs NAME)
  set(multiValueArgs SOURCES HEADERS DEPENDS INCLUDE_DIRECTORIES)
  cmake_parse_arguments(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(library valhalla-${MODULE_NAME})
  add_library(${library} OBJECT ${MODULE_SOURCES} ${MODULE_HEADERS})
  add_library(valhalla::${MODULE_NAME} ALIAS ${library})
  target_compile_definitions(${library}
    PUBLIC
      $<$<BOOL:${MSVC}>:VC_EXTRALEAN;WIN32_LEAN_AND_MEAN;NOMINMAX;NOGDI>
      RAPIDJSON_HAS_STDSTRING
      HAS_REMOTE_API=0
      AUTO_DOWNLOAD=0
    PRIVATE
      $<$<BOOL:${LOGGING_LEVEL}>:LOGGING_LEVEL_${LOGGING_LEVEL}>)
  target_include_directories(${library} ${MODULE_INCLUDE_DIRECTORIES}
     PRIVATE
       ${VALHALLA_BINARY_DIR}
       ${VALHALLA_BINARY_DIR}/valhalla)

  if(ENABLE_COVERAGE)
    target_compile_options(${library} PUBLIC -ftest-coverage -fprofile-arcs)
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
  set(libvalhalla_link_objects "${libvalhalla_link_objects};$<TARGET_OBJECTS:${library}>" CACHE INTERNAL "")
  set(libvalhalla_link_libraries "${libvalhalla_link_libraries};${external_module_depends}" CACHE INTERNAL "")
endfunction()

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
configure_file(../valhalla/valhalla.h.in ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla.h @ONLY)
configure_file(../valhalla/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/valhalla/config.h @ONLY)

set(valhalla_hdrs
    ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla.h
    ${VALHALLA_SOURCE_DIR}/valhalla/worker.h)

set(valhalla_src
    worker.cc
    ${CMAKE_CURRENT_BINARY_DIR}/valhalla/config.h
    ${valhalla_hdrs}
    ${libvalhalla_link_objects})

if (ENABLE_SERVICES)
  set(valahlla_hdrs ${valhalla_hdrs} ${VALHALLA_SOURCE_DIR}/valhalla/tile_server.h)
  set(valhalla_src ${valhalla_src} tile_server.cc)
endif()

add_library(valhalla ${valhalla_src})

target_compile_definitions(valhalla
  PUBLIC
    $<$<BOOL:${MSVC}>:VC_EXTRALEAN;WIN32_LEAN_AND_MEAN;NOMINMAX;NOGDI>
    RAPIDJSON_HAS_STDSTRING  # this is a part of the workaround for OBJECT libraries,
                             # because ${libvalhalla_compile_definitions} must be private.
                             # To be removed with ${libvalhalla_compile_definitions} below
    HAS_REMOTE_API=0
    AUTO_DOWNLOAD=0
  PRIVATE
    ${libvalhalla_compile_definitions})

target_compile_options(valhalla
  PUBLIC
    ${libvalhalla_compile_options}
  PRIVATE
  $<$<BOOL:${ENABLE_COMPILER_WARNINGS}>:-Wall -Wextra -Werror>
  )

target_include_directories(valhalla
  PUBLIC
    ${VALHALLA_SOURCE_DIR}
    ${VALHALLA_SOURCE_DIR}/valhalla # TODO: this path must be removed and changed to #include <valhalla/...> in headers
    ${VALHALLA_BINARY_DIR}
    ${VALHALLA_BINARY_DIR}/valhalla  # TODO: this path must be removed and changed to #include <valhalla/...> in headers
    ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
    ${VALHALLA_SOURCE_DIR}/third_party/date/include 
    $<$<BOOL:${MSVC}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
  PRIVATE
    ${libvalhalla_include_directories})

target_link_libraries(valhalla
  PUBLIC
    ${libvalhalla_link_libraries}
  PRIVATE
    $<$<BOOL:${ENABLE_COVERAGE}>:gcov>
    Threads::Threads)

set_target_properties(valhalla PROPERTIES
  VERSION "${VERSION}"
  SOVERSION "${VALHALLA_VERSION_MAJOR}")

set(LIBVALHALLA_SO_LINK ${CMAKE_SHARED_LIBRARY_PREFIX}valhalla${CMAKE_SHARED_LIBRARY_SUFFIX})

install(TARGETS valhalla
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shared NAMELINK_SKIP
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT development)

if(BUILD_SHARED_LIBS)
  install(FILES ${VALHALLA_SOURCE_DIR}/COPYING ${VALHALLA_SOURCE_DIR}/CHANGELOG.md
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/libvalhalla0"
    COMPONENT shared)
else()
  if(UNIX)
    add_custom_target(${LIBVALHALLA_SO_LINK} ALL
      COMMAND ${CMAKE_COMMAND} -E create_symlink "${LIBVALHALLA_SO_LINK}.${VALHALLA_VERSION_MAJOR}" ${LIBVALHALLA_SO_LINK})
  endif()
endif()

install(FILES ${VALHALLA_SOURCE_DIR}/COPYING ${VALHALLA_SOURCE_DIR}/CHANGELOG.md ${VALHALLA_SOURCE_DIR}/README.md
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/libvalhalla-dev"
  COMPONENT development)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${LIBVALHALLA_SO_LINK}
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
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
  install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} --build . --target libvalhalla.pc OUTPUT_QUIET ERROR_VARIABLE _err RESULT_VARIABLE _res)
  if(NOT \${_res} EQUAL 0)
    message(FATAL_ERROR \"Configuring libvalhalla.pc failed: \${_err}\")
  endif()")
  install(FILES ${VALHALLA_BINARY_DIR}/libvalhalla.pc
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
    COMPONENT development)
endif()
