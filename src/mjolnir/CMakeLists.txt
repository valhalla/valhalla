find_package(SQLite3 REQUIRED)
find_package(SpatiaLite REQUIRED)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/admin_lua_proc.h
  COMMAND ${CMAKE_COMMAND} -P ${VALHALLA_SOURCE_DIR}/cmake/Binary2Header.cmake
      ${VALHALLA_SOURCE_DIR}/lua/admin.lua
      ${CMAKE_CURRENT_BINARY_DIR}/admin_lua_proc.h
      --variable-name lua_admin_lua
  WORKING_DIRECTORY ${VALHALLA_SOURCE_DIR}
  COMMENT "Compiling lua/admin.lua to admin_lua_proc.h"
  DEPENDS ${VALHALLA_SOURCE_DIR}/lua/admin.lua
  VERBATIM)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/graph_lua_proc.h
  COMMAND ${CMAKE_COMMAND} -P ${VALHALLA_SOURCE_DIR}/cmake/Binary2Header.cmake
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
  complexrestrictionbuilder.cc
  countryaccess.cc
  dataquality.cc
  directededgebuilder.cc
  graphtilebuilder.cc
  edgeinfobuilder.cc
  uniquenames.cc
  ferry_connections.cc
  graphbuilder.cc
  graphenhancer.cc
  graphvalidator.cc
  hierarchybuilder.cc
  linkclassification.cc
  luatagtransform.cc
  node_expander.cc
  osmaccess.cc
  osmadmin.cc
  osmnode.cc
  osmpbfparser.cc
  osmaccessrestriction.cc
  osmrestriction.cc
  osmway.cc
  pbfadminparser.cc
  pbfgraphparser.cc
  restrictionbuilder.cc
  shortcutbuilder.cc
  transitbuilder.cc
  util.cc
  validatetransit.cc)

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
  DEPENDS
    valhalla::protobuf
    Spatialite::Spatialite
    SQLite3::SQLite3
    Lua::Lua
    ZLIB::ZLIB
    Threads::Threads)