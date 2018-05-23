file(GLOB locale_jsons ${VALHALLA_SOURCE_DIR}/locales *.json)
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/locales.h
  COMMAND ${CMAKE_COMMAND} -P cmake/Binary2Header.cmake ${VALHALLA_SOURCE_DIR}/locales/ ${CMAKE_CURRENT_BINARY_DIR}/locales.h --locales
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
  COMMENT "Compiling locales/*.json to locales.h"
  DEPENDS ${locale_jsons}
  VERBATIM)

set(headers
  ${CMAKE_CURRENT_BINARY_DIR}/locales.h

  ${VALHALLA_SOURCE_DIR}/valhalla/odin/worker.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/directionsbuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/maneuversbuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/narrative_dictionary.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/narrative_builder_factory.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/narrativebuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/enhancedtrippath.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/maneuver.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/sign.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/signs.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/util.h
  ${VALHALLA_SOURCE_DIR}/valhalla/odin/transitrouteinfo.h)

set(sources
  directionsbuilder.cc
  maneuversbuilder.cc
  narrative_dictionary.cc
  narrative_builder_factory.cc
  narrativebuilder.cc
  enhancedtrippath.cc
  maneuver.cc
  sign.cc
  signs.cc
  util.cc
  transitrouteinfo.cc
  worker.cc)

valhalla_module(NAME odin
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    valhalla::tyr
    valhalla::protobuf)
