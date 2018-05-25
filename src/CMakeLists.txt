include(FindPkgConfig)
include(GNUInstallDirs)

## Set relative to the current source and binary paths
set(LIBVALHALLA_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
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
if(VALHALLA_VERSION_MAJOR)
  set(VERSION "${VALHALLA_VERSION_MAJOR}")
  if(VALHALLA_VERSION_MINOR)
    set(VERSION "${VERSION}.${VALHALLA_VERSION_MINOR}")
    if(VALHALLA_VERSION_PATCH)
      set(VERSION "${VERSION}.${VALHALLA_VERSION_PATCH}")
    endif()
  endif()
else()
  message(FATAL_ERROR "No Valhalla major version")
endif()

## Configure imported libraries
set(boost_components
  date_time
  filesystem
  iostreams
  regex
  system)
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  list(APPEND boost_components zlib)
endif()
find_package(Boost 1.51 REQUIRED COMPONENTS ${boost_components})

find_package(CURL REQUIRED)
add_library(CURL::CURL INTERFACE IMPORTED)
set_target_properties(CURL::CURL PROPERTIES
  INTERFACE_LINK_LIBRARIES "${CURL_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${CURL_INCLUDE_DIR}"
  INTERFACE_COMPILE_DEFINITIONS CURL_STATICLIB)

find_package(Lua REQUIRED)
add_library(Lua::Lua INTERFACE IMPORTED)
set_target_properties(Lua::Lua PROPERTIES
  INTERFACE_LINK_LIBRARIES "${LUA_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${LUA_INCLUDE_DIR}")

find_package(ZLIB REQUIRED)
find_package(LZ4 REQUIRED)
find_package(Threads REQUIRED)

add_library(libprime_server INTERFACE IMPORTED)
if(ENABLE_SERVICES)
  pkg_check_modules(libprime_server REQUIRED libprime_server>=0.6.3)
  set_target_properties(libprime_server PROPERTIES
    INTERFACE_LINK_LIBRARIES "${libprime_server_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${libprime_server_INCLUDE_DIRS}"
    INTERFACE_COMPILE_DEFINITIONS HAVE_HTTP)
endif()

## Define libvalhalla sub-modules
function(valhalla_module)
  set(oneValueArgs NAME)
  set(multiValueArgs SOURCES HEADERS DEPENDS INCLUDE_DIRECTORIES)
  cmake_parse_arguments(MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  set(library valhalla-${MODULE_NAME})
  add_library(${library} STATIC ${MODULE_SOURCES} ${MODULE_HEADERS})
  add_library(valhalla::${MODULE_NAME} ALIAS ${library})
  target_compile_definitions(${library}
    PUBLIC
      RAPIDJSON_HAS_STDSTRING
      $<$<BOOL:${LOGGING_LEVEL}>:LOGGING_LEVEL_${LOGGING_LEVEL}>)
  target_include_directories(${library} ${MODULE_INCLUDE_DIRECTORIES}
     PRIVATE
       ${LIBVALHALLA_BINARY_DIR}
       ${LIBVALHALLA_BINARY_DIR}/valhalla)
  target_link_libraries(${library} ${MODULE_DEPENDS})

  if(ENABLE_COVERAGE)
    target_compile_options(${library} PUBLIC -ftest-coverage -fprofile-arcs)
  endif()

  install(FILES ${MODULE_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${MODULE_NAME}
    COMPONENT development)

  if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
    set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
  endif()
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

configure_file(../valhalla/valhalla.h.in ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla.h @ONLY)
configure_file(../valhalla/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/valhalla/config.h @ONLY)

set(valhalla_hdrs
  ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla.h
  ${CMAKE_CURRENT_BINARY_DIR}/valhalla/config.h

  ${VALHALLA_SOURCE_DIR}/valhalla/exception.h
  ${VALHALLA_SOURCE_DIR}/valhalla/worker.h)

add_library(valhalla worker.cc ${valhalla_hdrs})

target_compile_definitions(valhalla PRIVATE
  $<$<BOOL:${MSVC}>:"VC_EXTRALEAN;WIN32_LEAN_AND_MEAN;NOMINMAX;NOGDI">)

target_include_directories(valhalla
  PUBLIC
    ${VALHALLA_SOURCE_DIR}
    ${VALHALLA_SOURCE_DIR}/valhalla # TODO: this path must be removed and changed to #include <valhalla/...> in headers
    ${LIBVALHALLA_BINARY_DIR}
    ${LIBVALHALLA_BINARY_DIR}/valhalla  # TODO: this path must be removed and changed to #include <valhalla/...> in headers
    )

target_link_libraries(valhalla
  $<$<BOOL:${ENABLE_DATA_TOOLS}>:valhalla::mjolnir>
  valhalla::baldr
  valhalla::tyr
  valhalla::midgard
  valhalla::meili
  valhalla::sif
  valhalla::skadi
  valhalla::protobuf
  $<$<BOOL:${ENABLE_COVERAGE}>:gcov>
  Threads::Threads)

set_target_properties(valhalla PROPERTIES
  VERSION "${VERSION}"
  SOVERSION "${VALHALLA_VERSION_MAJOR}")

set(LIBVALHALLA_SO_LINK ${CMAKE_SHARED_LIBRARY_PREFIX}valhalla${CMAKE_SHARED_LIBRARY_SUFFIX})

install(TARGETS valhalla
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT shared NAMELINK_SKIP
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development)

if(BUILD_SHARED_LIBS)
  install(FILES ${VALHALLA_SOURCE_DIR}/COPYING ${VALHALLA_SOURCE_DIR}/ChangeLog
    DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/libvalhalla0"
    COMPONENT shared)
else()
  add_custom_command(TARGET valhalla POST_BUILD
    COMMAND ${CMAKE_AR} rc $<TARGET_FILE:valhalla>
      $<$<BOOL:${ENABLE_DATA_TOOLS}>:$<TARGET_FILE:valhalla::mjolnir>>
      $<TARGET_FILE:valhalla::baldr>
      $<TARGET_FILE:valhalla::loki>
      $<TARGET_FILE:valhalla::meili>
      $<TARGET_FILE:valhalla::midgard>
      $<TARGET_FILE:valhalla::odin>
      $<TARGET_FILE:valhalla::sif>
      $<TARGET_FILE:valhalla::skadi>
      $<TARGET_FILE:valhalla::thor>
      $<TARGET_FILE:valhalla::tyr>)

  add_custom_target(${LIBVALHALLA_SO_LINK} ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink "${LIBVALHALLA_SO_LINK}.${VALHALLA_VERSION_MAJOR}" ${LIBVALHALLA_SO_LINK})
endif()

install(FILES ${VALHALLA_SOURCE_DIR}/COPYING ${VALHALLA_SOURCE_DIR}/ChangeLog ${VALHALLA_SOURCE_DIR}/README.md
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/doc/libvalhalla-dev"
  COMPONENT development)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${LIBVALHALLA_SO_LINK}
  DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  COMPONENT development)

install(FILES ${valhalla_hdrs}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/valhalla"
  COMPONENT development)


## pkg-config file
set(prefix ${CMAKE_INSTALL_PREFIX})
set(exec_prefix ${CMAKE_INSTALL_PREFIX})
set(libdir ${CMAKE_INSTALL_LIBDIR})
set(includedir ${CMAKE_INSTALL_INCLUDEDIR}/valhalla)
configure_file(../libvalhalla.pc.in libvalhalla.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libvalhalla.pc
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig
  COMPONENT development)
