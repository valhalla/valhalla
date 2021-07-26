
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
  adminbuilder.cc
  complexrestrictionbuilder.cc
  countryaccess.cc
  directededgebuilder.cc
  edgeinfobuilder.cc
  ferry_connections.cc
  graphfilter.cc
  linkclassification.cc
  node_expander.cc
  osmdata.cc
  osmpbfparser.cc
  osmaccessrestriction.cc
  osmrestriction.cc
  osmway.cc
  pbfadminparser.cc
  restrictionbuilder.cc
  servicedays.cc
  speed_assigner.h
  timeparsing.cc
  util.cc)

set (sources_with_warnings
  admin.cc
  bssbuilder.cc
  dataquality.cc
  elevationbuilder.cc
  graphbuilder.cc
  graphenhancer.cc
  graphtilebuilder.cc
  graphvalidator.cc
  hierarchybuilder.cc
  luatagtransform.cc
  pbfgraphparser.cc
  shortcutbuilder.cc
  transitbuilder.cc
  validatetransit.cc)

if (UNIX AND ENABLE_SINGLE_FILES_WERROR)
  # Enables stricter compiler checks on a file-by-file basis
  # which allows us to migrate piecemeal
  set_source_files_properties(
          ${sources}
          PROPERTIES COMPILE_FLAGS "-Werror -Wall -Wextra -Wno-missing-field-initializers -Wno-unused-parameter")
endif ()

valhalla_module(NAME mjolnir
  SOURCES ${sources} ${sources_with_warnings}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      ${VALHALLA_SOURCE_DIR}/third_party/date/include
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
      ${PROTOBUF_INCLUDE_DIR}
   PRIVATE
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}/valhalla
  DEPENDS
    valhalla::proto
    valhalla::baldr
    SpatiaLite::SpatiaLite
    SQLite3::SQLite3
    Lua::Lua
    Threads::Threads
    ZLIB::ZLIB
    robin_hood::robin_hood)
