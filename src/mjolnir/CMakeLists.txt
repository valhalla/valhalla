
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/admin_lua_proc.h
  COMMAND ${CMAKE_COMMAND} -P ${VALHALLA_SOURCE_DIR}/cmake/ValhallaBin2Header.cmake
      ${VALHALLA_SOURCE_DIR}/lua/admin.lua
      ${CMAKE_CURRENT_BINARY_DIR}/admin_lua_proc.h
      --variable-name lua_admin_lua
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
  COMMENT "Compiling lua/admin.lua to admin_lua_proc.h"
  DEPENDS ${VALHALLA_SOURCE_DIR}/lua/admin.lua
  VERBATIM)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/graph_lua_proc.h
  COMMAND ${CMAKE_COMMAND} -P ${VALHALLA_SOURCE_DIR}/cmake/ValhallaBin2Header.cmake
      ${VALHALLA_SOURCE_DIR}/lua/graph.lua
      ${CMAKE_CURRENT_BINARY_DIR}/graph_lua_proc.h
      --variable-name lua_graph_lua
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
  COMMENT "Compiling lua/graph.lua to graph_lua_proc.h"
  DEPENDS ${VALHALLA_SOURCE_DIR}/lua/graph.lua
  VERBATIM)

file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/*.h)

set(sources
  ${CMAKE_CURRENT_BINARY_DIR}/graph_lua_proc.h
  ${CMAKE_CURRENT_BINARY_DIR}/admin_lua_proc.h
  admin.cc
  adminbuilder.cc
  bssbuilder.cc
  complexrestrictionbuilder.cc
  convert_transit.cc
  countryaccess.cc
  dataquality.cc
  directededgebuilder.cc
  edgeinfobuilder.cc
  elevationbuilder.cc
  ferry_connections.cc
  graphbuilder.cc
  graphenhancer.cc
  graphfilter.cc
  graphtilebuilder.cc
  graphvalidator.cc
  hierarchybuilder.cc
  ingest_transit.cc
  landmarks.cc
  linkclassification.cc
  luatagtransform.cc
  node_expander.cc
  osmdata.cc
  osmpbfparser.cc
  osmaccessrestriction.cc
  osmrestriction.cc
  osmway.cc
  pbfadminparser.cc
  pbfgraphparser.cc
  restrictionbuilder.cc
  servicedays.cc
  shortcutbuilder.cc
  speed_assigner.h
  timeparsing.cc
  transitbuilder.cc
  util.cc
  validatetransit.cc)

set(system_includes
  ${date_include_dir}
  ${rapidjson_include_dir}
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>
  ${PROTOBUF_INCLUDE_DIR})
if(APPLE)
  list(APPEND system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include/date)
endif()

valhalla_module(NAME mjolnir
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
   PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}/valhalla
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/just_gtfs/include
      ${robinhoodhashing_include_dir}
  DEPENDS
    valhalla::proto
    valhalla::baldr
    PkgConfig::SpatiaLite
    SQLite3::SQLite3
    Boost::boost
    PkgConfig::LuaJIT
    Threads::Threads
    PkgConfig::ZLIB)
