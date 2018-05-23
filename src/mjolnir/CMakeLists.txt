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

set(headers
  ${CMAKE_CURRENT_BINARY_DIR}/graph_lua_proc.h
  ${CMAKE_CURRENT_BINARY_DIR}/admin_lua_proc.h

  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/admin.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/countryaccess.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/complexrestrictionbuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/dataquality.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/directededgebuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/graphtilebuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/edgeinfobuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/uniquenames.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/ferry_connections.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/graphbuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/graphenhancer.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/graphvalidator.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/hierarchybuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/idtable.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/linkclassification.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/luatagtransform.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/node_expander.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/osmaccess.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/osmadmin.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/osmdata.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/osmnode.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/osmpbfparser.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/osmaccessrestriction.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/osmrestriction.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/osmway.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/pbfadminparser.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/pbfgraphparser.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/restrictionbuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/shortcutbuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/transitbuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/util.h
  ${VALHALLA_SOURCE_DIR}/valhalla/mjolnir/validatetransit.h)

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