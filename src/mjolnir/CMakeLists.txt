message(STATUS "Configuring lua/admin.lua to generate admin_lua_proc.h")
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/valhalla/admin_lua_proc.h
  COMMAND ${CMAKE_COMMAND} -P cmake/Binary2Header.cmake lua/admin.lua ${CMAKE_BINARY_DIR}/valhalla/admin_lua_proc.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling lua/admin.lua to admin_lua_proc.h"
  DEPENDS ${CMAKE_SOURCE_DIR}/lua/admin.lua
  VERBATIM)

message(STATUS "Configuring lua/graph.lua to generate graph_lua_proc.h")
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/valhalla/graph_lua_proc.h
  COMMAND ${CMAKE_COMMAND} -P cmake/Binary2Header.cmake lua/graph.lua ${CMAKE_BINARY_DIR}/valhalla/graph_lua_proc.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling lua/graph.lua to graph_lua_proc.h"
  DEPENDS ${CMAKE_SOURCE_DIR}/lua/graph.lua
  VERBATIM)

set(headers
  ${CMAKE_BINARY_DIR}/valhalla/graph_lua_proc.h
  ${CMAKE_BINARY_DIR}/valhalla/admin_lua_proc.h

  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/admin.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/countryaccess.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/complexrestrictionbuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/dataquality.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/directededgebuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/graphtilebuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/edgeinfobuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/uniquenames.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/ferry_connections.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/graphbuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/graphenhancer.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/graphvalidator.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/hierarchybuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/idtable.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/linkclassification.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/luatagtransform.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/node_expander.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/osmaccess.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/osmadmin.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/osmdata.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/osmnode.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/osmpbfparser.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/osmaccessrestriction.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/osmrestriction.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/osmway.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/pbfadminparser.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/pbfgraphparser.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/restrictionbuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/shortcutbuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/transitbuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/util.h
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/validatetransit.h)

set(sources
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
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_BINARY_DIR}
      ${CMAKE_BINARY_DIR}/valhalla
  DEPENDS
    Spatialite::Spatialite
    SQLite3::SQLite3
    Lua::Lua
    Threads::Threads
    ZLIB::ZLIB)