message(STATUS "Configuring locales/*.json to generate locales.h")
file(GLOB locale_jsons ${CMAKE_SOURCE_DIR}/locales *.json)
set(_locales_h ${CMAKE_BINARY_DIR}/valhalla/locales.h)
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/valhalla/locales.h
  COMMAND echo "#include <unordered_map>\\n" > ${_locales_h}
  COMMAND find . -name "*.json" -exec ${XXD} -i {} \; >> ${_locales_h}
  COMMAND echo "const std::unordered_map<std::string, std::string> locales_json = {" >> ${_locales_h}
  COMMAND find . -name "*.json" -printf "%f\\n" | sed s/.json// | sed "p;s@[./-]@_@g" | xargs -n 2 | awk "{ printf \"    {\\\"%s\\\", {__%s_json, __%s_json + __%s_json_len}},\\n\", $1, $2, $2, $2 }"  >> ${_locales_h}
  COMMAND echo "};\\n" >> ${_locales_h}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/locales
  COMMENT "Compiling lua/admin.lua to admin_lua_proc.h"
  DEPENDS ${locale_jsons}
  VERBATIM)

set(headers
  ${CMAKE_BINARY_DIR}/valhalla/locales.h

  ${CMAKE_SOURCE_DIR}/valhalla/odin/worker.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/directionsbuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/maneuversbuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/narrative_dictionary.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/narrative_builder_factory.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/narrativebuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/enhancedtrippath.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/maneuver.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/sign.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/signs.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/util.h
  ${CMAKE_SOURCE_DIR}/valhalla/odin/transitrouteinfo.h)

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
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_BINARY_DIR}
      ${CMAKE_BINARY_DIR}/valhalla
  DEPENDS
    valhalla::tyr
    valhalla::protobuf)
