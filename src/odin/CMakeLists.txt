file(GLOB locale_jsons ${VALHALLA_SOURCE_DIR}/locales *.json)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/locales.h
  COMMAND ${CMAKE_COMMAND} -DMSVC=${MSVC} -P cmake/Binary2Header.cmake ${VALHALLA_SOURCE_DIR}/locales/ ${CMAKE_CURRENT_BINARY_DIR}/locales.h --locales
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
  COMMENT "Compiling locales/*.json to locales.h"
  DEPENDS ${locale_jsons}
  VERBATIM)

file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/odin/*.h)

set(sources
  ${CMAKE_CURRENT_BINARY_DIR}/locales.h

  markup_formatter.cc
  narrative_dictionary.cc
  sign.cc
  signs.cc
  transitrouteinfo.cc
  worker.cc)

set(sources_with_warnings
  directionsbuilder.cc
  enhancedtrippath.cc
  maneuver.cc
  maneuversbuilder.cc
  narrative_builder_factory.cc
  narrativebuilder.cc
  util.cc)

# treat date library as system
set(system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include)
if(APPLE)
  list(APPEND system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include/date)
endif()

valhalla_module(NAME odin
  SOURCES ${sources}
  SOURCES_WITH_WARNINGS ${sources_with_warnings}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
      ${CMAKE_CURRENT_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
  DEPENDS
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost
    libprime_server)
