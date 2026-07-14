file(GLOB locale_pos ${VALHALLA_SOURCE_DIR}/locales/*.po)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/locales.h
  COMMAND ${PYTHON_INTERPRETER} ${VALHALLA_SOURCE_DIR}/locales/po_tools.py po2json --out ${CMAKE_CURRENT_BINARY_DIR}/locales
  COMMAND ${CMAKE_COMMAND} -DMSVC=${MSVC} -P ${VALHALLA_SOURCE_DIR}/cmake/ValhallaBin2Header.cmake ${CMAKE_CURRENT_BINARY_DIR}/locales/ ${CMAKE_CURRENT_BINARY_DIR}/locales.h --locales
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
  COMMENT "Generating locale JSONs from locales/*.po and compiling to locales.h"
  DEPENDS ${locale_pos} ${VALHALLA_SOURCE_DIR}/locales/en-US.json ${VALHALLA_SOURCE_DIR}/locales/po_tools.py
  VERBATIM)

file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/odin/*.h)

set(sources
  ${CMAKE_CURRENT_BINARY_DIR}/locales.h
  markup_formatter.cc
  narrative_dictionary.cc
  sign.cc
  signs.cc
  transitrouteinfo.cc
  worker.cc
  directionsbuilder.cc
  enhancedtrippath.cc
  maneuver.cc
  maneuversbuilder.cc
  narrative_builder_factory.cc
  narrativebuilder.cc
  util.cc)


set(system_includes
  ${date_include_dir}
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>)

valhalla_module(NAME odin
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
    PRIVATE
      ${rapidjson_include_dir}
  DEPENDS
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost
    ${libprime_server_targets})
