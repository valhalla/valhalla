# Minimal CMake configuration for Valhalla
#
# Builds libvalhalla and minimal collection of programs.
#
# This is NOT equivalent to the official Valhalla build configuration based on GNU Autotools.
# This is NOT suitable for building complete Valhalla suite.
# This is secondary build configuration provided for convenient development
# on Windows and using CMake-enabled IDEs.
#
cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(valhalla LANGUAGES CXX C)

include(FindPkgConfig)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

option(ENABLE_CCACHE "Speed up incremental rebuilds via ccache" ON)
option(ENABLE_HTTP "Enable HTTP" ON)
option(ENABLE_DATA_TOOLS "Enable Valhalla data tools" ON)
option(ENABLE_SERVICES "Enable Valhalla services" ON)
option(ENABLE_PYTHON_BINDINGS "Enable Python bindings" OFF)


## Configure imported libraries
set(boost_components
  date_time
  filesystem
  iostreams
  program_options
  regex
  system
  thread)
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
find_package(SQLite3 REQUIRED)
find_package(SpatiaLite)
find_package(GEOS)

add_library(libprime_server INTERFACE IMPORTED)
if(ENABLE_HTTP)
  pkg_check_modules(libprime_server libprime_server>=0.6.3)
  if(libprime_server_FOUND)
    set_target_properties(libprime_server PROPERTIES
      INTERFACE_LINK_LIBRARIES "${libprime_server_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${libprime_server_INCLUDE_DIRS}"
      INTERFACE_COMPILE_DEFINITIONS HAVE_HTTP)
  endif()
endif()

if(ENABLE_CCACHE AND (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    message(STATUS "Using ccache to speed up incremental builds")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    set(ENV{CCACHE_CPP2} "true")
  endif()
endif()


## Custom commands for generated files
find_program(XXD xxd)
if (NOT XXD)
  message(FATAL_ERROR "xxd executable not found")
endif()

add_custom_command(OUTPUT valhalla/date_time_zonespec.h
  COMMAND ${XXD} -i -s `head -1 date_time/zonespec.csv | wc -c` date_time/zonespec.csv ${CMAKE_BINARY_DIR}/valhalla/date_time_zonespec.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling date_time/zonespec.csv to date_time_zonespec.h"
  DEPENDS date_time/zonespec.csv)

message(STATUS "Configuring lua/graph.lua to generate graph_lua_proc.h")
add_custom_command(OUTPUT valhalla/graph_lua_proc.h
  COMMAND ${XXD} -i lua/graph.lua ${CMAKE_BINARY_DIR}/valhalla/graph_lua_proc.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling lua/graph.lua to graph_lua_proc.h"
  DEPENDS lua/graph.lua
  VERBATIM)

message(STATUS "Configuring lua/admin.lua to generate admin_lua_proc.h")
add_custom_command(OUTPUT valhalla/admin_lua_proc.h
  COMMAND ${XXD} -i lua/admin.lua ${CMAKE_BINARY_DIR}/valhalla/admin_lua_proc.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling lua/admin.lua to admin_lua_proc.h"
  DEPENDS lua/admin.lua
  VERBATIM)

message(STATUS "Configuring locales/*.json to generate locales.h")
file(GLOB locale_jsons ${CMAKE_SOURCE_DIR}/locales *.json)
set(_locales_h ${CMAKE_BINARY_DIR}/valhalla/locales.h)
add_custom_command(OUTPUT valhalla/locales.h
  COMMAND echo "#include <unordered_map>\\n" > ${_locales_h}
  COMMAND find . -name *.json -exec ${XXD} -i {} \; >> ${_locales_h}
  COMMAND echo "const std::unordered_map<std::string, std::string> locales_json = {" >> ${_locales_h}
  COMMAND find . -name *.json -printf "%f\\n" | sed s/.json// | sed s@[./-]@_@g  | awk "{ printf \"    {\\\"%s\\\", {__%s_json, __%s_json + __%s_json_len}},\\n\", $1, $1, $1, $1 }"  >> ${_locales_h}
  COMMAND echo "};\\n" >> ${_locales_h}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/locales
  COMMENT "Compiling lua/admin.lua to admin_lua_proc.h"
  DEPENDS ${locale_jsons}
  VERBATIM)


## Protobuf
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(protobuff_srcs protobuff_hdrs
  proto/directions_options.proto
  proto/navigator.proto
  proto/route.proto
  proto/tripcommon.proto
  proto/tripdirections.proto
  proto/trippath.proto

  proto/fileformat.proto
  proto/osmformat.proto
  proto/segment.proto
  proto/tile.proto
  proto/transit.proto
  proto/transit_fetch.proto)


## libvalhalla
message(STATUS "Generating  ${CMAKE_BINARY_DIR}/valhalla/valhalla.h")
configure_file(${CMAKE_SOURCE_DIR}/valhalla/valhalla.h.in ${CMAKE_BINARY_DIR}/valhalla/valhalla.h @ONLY)

message(STATUS "Generating  ${CMAKE_BINARY_DIR}/valhalla/config.h")
configure_file(${CMAKE_SOURCE_DIR}/valhalla/config.h.cmake ${CMAKE_BINARY_DIR}/valhalla/config.h @ONLY)

message(STATUS "Configuring valhalla library target")
set(valhalla_hdrs
  ${CMAKE_SOURCE_DIR}/valhalla/valhalla.h
  ${CMAKE_SOURCE_DIR}/valhalla/exception.h
  ${CMAKE_SOURCE_DIR}/valhalla/worker.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/linesegment2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/tiles.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/gridded_data.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/polyline2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/obb2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/pointll.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/vector2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/constants.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/aabb2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/point2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/util.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/distanceapproximator.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/ellipse.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/sequence.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/shape_decoder.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/encoded.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/logging.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/accessrestriction.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/admin.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/admininfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/complexrestriction.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/connectivity_map.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/curler.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/datetime.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/directededge.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/double_bucket_queue.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/edge_elevation.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/edgeinfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/filesystem_utils.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphconstants.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphid.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphreader.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphtile.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/graphtileheader.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/json.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/nodeinfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/location.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/pathlocation.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/rapidjson_utils.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/sign.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/signinfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/tilehierarchy.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/turn.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetname.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetnames.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetnames_factory.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetname_us.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/streetnames_us.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/trafficassociation.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transitdeparture.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transitroute.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transitschedule.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transitstop.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/transittransfer.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/laneconnectivity.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter_us.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter_us_co.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter_us_tx.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/verbal_text_formatter_factory.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/reutil.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/edgetracker.h
  ${CMAKE_SOURCE_DIR}/valhalla/baldr/merge.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/costconstants.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/costfactory.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/autocost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/bicyclecost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/motorscootercost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/pedestriancost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/transitcost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/truckcost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/dynamiccost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/hierarchylimits.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/edgelabel.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/universal_cost.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/candidate_search.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/geometry_helpers.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/grid_range_query.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/emission_cost_model.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/transition_cost_model.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/priority_queue.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/routing.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/stateid.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/viterbi_search.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/topk_search.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/measurement.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/match_result.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/grid_traversal.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/state.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/map_matcher.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/map_matcher_factory.h
  ${CMAKE_SOURCE_DIR}/valhalla/meili/traffic_segment_matcher.h
  ${CMAKE_SOURCE_DIR}/valhalla/skadi/sample.h
  ${CMAKE_SOURCE_DIR}/valhalla/skadi/util.h
  ${CMAKE_SOURCE_DIR}/valhalla/loki/worker.h
  ${CMAKE_SOURCE_DIR}/valhalla/loki/search.h
  ${CMAKE_SOURCE_DIR}/valhalla/loki/node_search.h
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
  ${CMAKE_SOURCE_DIR}/valhalla/odin/transitrouteinfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/worker.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/astar.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/astarheuristic.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/bidirectional_astar.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/costmatrix.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/edgestatus.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/isochrone.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/optimizer.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/map_matcher.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/match_result.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/multimodal.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/pathalgorithm.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/pathinfo.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/route_matcher.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/trippathbuilder.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/attributes_controller.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/trafficalgorithm.h
  ${CMAKE_SOURCE_DIR}/valhalla/thor/timedistancematrix.h
  ${CMAKE_SOURCE_DIR}/valhalla/tyr/serializers.h
  ${CMAKE_SOURCE_DIR}/valhalla/tyr/navigator.h
  ${CMAKE_SOURCE_DIR}/valhalla/tyr/actor.h
  ${CMAKE_BINARY_DIR}/valhalla/date_time_zonespec.h
  ${CMAKE_BINARY_DIR}/valhalla/locales.h)

set(valhalla_srcs
  ${CMAKE_SOURCE_DIR}/src/worker.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/linesegment2.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/tiles.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/gridded_data.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/polyline2.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/obb2.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/pointll.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/aabb2.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/point2.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/util.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/ellipse.cc
  ${CMAKE_SOURCE_DIR}/src/midgard/logging.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/accessrestriction.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/admin.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/connectivity_map.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/datetime.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/directededge.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/edge_elevation.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/edgeinfo.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/graphid.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/graphreader.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/graphtile.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/graphtileheader.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/edgetracker.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/merge.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/nodeinfo.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/location.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/pathlocation.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/tilehierarchy.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/turn.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/streetname.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/streetnames.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/streetnames_factory.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/streetname_us.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/streetnames_us.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/transitdeparture.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/transitroute.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/transitschedule.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/transitstop.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/transittransfer.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/laneconnectivity.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/verbal_text_formatter.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/verbal_text_formatter_us.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/verbal_text_formatter_us_co.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/verbal_text_formatter_us_tx.cc
  ${CMAKE_SOURCE_DIR}/src/baldr/verbal_text_formatter_factory.cc
  ${CMAKE_SOURCE_DIR}/src/sif/autocost.cc
  ${CMAKE_SOURCE_DIR}/src/sif/bicyclecost.cc
  ${CMAKE_SOURCE_DIR}/src/sif/motorscootercost.cc
  ${CMAKE_SOURCE_DIR}/src/sif/pedestriancost.cc
  ${CMAKE_SOURCE_DIR}/src/sif/transitcost.cc
  ${CMAKE_SOURCE_DIR}/src/sif/truckcost.cc
  ${CMAKE_SOURCE_DIR}/src/sif/dynamiccost.cc
  ${CMAKE_SOURCE_DIR}/src/meili/universal_cost.cc
  ${CMAKE_SOURCE_DIR}/src/meili/viterbi_search.cc
  ${CMAKE_SOURCE_DIR}/src/meili/topk_search.cc
  ${CMAKE_SOURCE_DIR}/src/meili/routing.cc
  ${CMAKE_SOURCE_DIR}/src/meili/candidate_search.cc
  ${CMAKE_SOURCE_DIR}/src/meili/transition_cost_model.cc
  ${CMAKE_SOURCE_DIR}/src/meili/map_matcher.cc
  ${CMAKE_SOURCE_DIR}/src/meili/map_matcher_factory.cc
  ${CMAKE_SOURCE_DIR}/src/meili/match_route.cc
  ${CMAKE_SOURCE_DIR}/src/meili/traffic_segment_matcher.cc
  ${CMAKE_SOURCE_DIR}/src/skadi/sample.cc
  ${CMAKE_SOURCE_DIR}/src/skadi/util.cc
  ${CMAKE_SOURCE_DIR}/src/loki/search.cc
  ${CMAKE_SOURCE_DIR}/src/loki/worker.cc
  ${CMAKE_SOURCE_DIR}/src/loki/height_action.cc
  ${CMAKE_SOURCE_DIR}/src/loki/locate_action.cc
  ${CMAKE_SOURCE_DIR}/src/loki/route_action.cc
  ${CMAKE_SOURCE_DIR}/src/loki/matrix_action.cc
  ${CMAKE_SOURCE_DIR}/src/loki/isochrone_action.cc
  ${CMAKE_SOURCE_DIR}/src/loki/trace_route_action.cc
  ${CMAKE_SOURCE_DIR}/src/loki/transit_available_action.cc
  ${CMAKE_SOURCE_DIR}/src/loki/node_search.cc
  ${CMAKE_SOURCE_DIR}/src/odin/directionsbuilder.cc
  ${CMAKE_SOURCE_DIR}/src/odin/maneuversbuilder.cc
  ${CMAKE_SOURCE_DIR}/src/odin/narrative_dictionary.cc
  ${CMAKE_SOURCE_DIR}/src/odin/narrative_builder_factory.cc
  ${CMAKE_SOURCE_DIR}/src/odin/narrativebuilder.cc
  ${CMAKE_SOURCE_DIR}/src/odin/enhancedtrippath.cc
  ${CMAKE_SOURCE_DIR}/src/odin/maneuver.cc
  ${CMAKE_SOURCE_DIR}/src/odin/sign.cc
  ${CMAKE_SOURCE_DIR}/src/odin/signs.cc
  ${CMAKE_SOURCE_DIR}/src/odin/util.cc
  ${CMAKE_SOURCE_DIR}/src/odin/transitrouteinfo.cc
  ${CMAKE_SOURCE_DIR}/src/odin/worker.cc
  ${CMAKE_SOURCE_DIR}/src/thor/astar.cc
  ${CMAKE_SOURCE_DIR}/src/thor/bidirectional_astar.cc
  ${CMAKE_SOURCE_DIR}/src/thor/costmatrix.cc
  ${CMAKE_SOURCE_DIR}/src/thor/isochrone.cc
  ${CMAKE_SOURCE_DIR}/src/thor/map_matcher.cc
  ${CMAKE_SOURCE_DIR}/src/thor/multimodal.cc
  ${CMAKE_SOURCE_DIR}/src/thor/optimizer.cc
  ${CMAKE_SOURCE_DIR}/src/thor/trippathbuilder.cc
  ${CMAKE_SOURCE_DIR}/src/thor/attributes_controller.cc
  ${CMAKE_SOURCE_DIR}/src/thor/route_matcher.cc
  ${CMAKE_SOURCE_DIR}/src/thor/timedep_forward.cc
  ${CMAKE_SOURCE_DIR}/src/thor/timedep_reverse.cc
  ${CMAKE_SOURCE_DIR}/src/thor/trafficalgorithm.cc
  ${CMAKE_SOURCE_DIR}/src/thor/timedistancematrix.cc
  ${CMAKE_SOURCE_DIR}/src/thor/worker.cc
  ${CMAKE_SOURCE_DIR}/src/thor/isochrone_action.cc
  ${CMAKE_SOURCE_DIR}/src/thor/matrix_action.cc
  ${CMAKE_SOURCE_DIR}/src/thor/optimized_route_action.cc
  ${CMAKE_SOURCE_DIR}/src/thor/route_action.cc
  ${CMAKE_SOURCE_DIR}/src/thor/trace_attributes_action.cc
  ${CMAKE_SOURCE_DIR}/src/thor/trace_route_action.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/serializers.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/isochrone_serializer.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/matrix_serializer.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/height_serializer.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/locate_serializer.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/route_serializer.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/transit_available_serializer.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/trace_serializer.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/navigator.cc
  ${CMAKE_SOURCE_DIR}/src/tyr/actor.cc)

set(valhalla_data_tools_hdrs
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
  ${CMAKE_SOURCE_DIR}/valhalla/mjolnir/validatetransit.h
  ${CMAKE_BINARY_DIR}/valhalla/graph_lua_proc.h
  ${CMAKE_BINARY_DIR}/valhalla/admin_lua_proc.h)

set(valhalla_data_tools_srcs
  ${CMAKE_SOURCE_DIR}/src/mjolnir/admin.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/complexrestrictionbuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/countryaccess.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/dataquality.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/directededgebuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/graphtilebuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/edgeinfobuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/uniquenames.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/ferry_connections.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/graphbuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/graphenhancer.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/graphvalidator.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/hierarchybuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/linkclassification.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/luatagtransform.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/node_expander.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/osmaccess.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/osmadmin.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/osmnode.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/osmpbfparser.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/osmaccessrestriction.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/osmrestriction.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/osmway.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/pbfadminparser.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/pbfgraphparser.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/restrictionbuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/shortcutbuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/transitbuilder.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/util.cc
  ${CMAKE_SOURCE_DIR}/src/mjolnir/validatetransit.cc)

add_library(valhalla
  ${valhalla_srcs}
  ${valhalla_hdrs}
  ${valhalla_data_tools_srcs}
  ${valhalla_data_tools_hdrs}
  ${protobuff_srcs}
  ${protobuff_hdrs})

target_link_libraries(valhalla
  libprime_server
  protobuf::libprotobuf
  Boost::filesystem
  Boost::iostreams
  Boost::thread
  Spatialite::Spatialite
  SQLite3::SQLite3
  Lua::Lua
  CURL::CURL
  LZ4::LZ4
  ZLIB::ZLIB)

target_include_directories(valhalla
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
    ${CMAKE_BINARY_DIR}
    ${CMAKE_BINARY_DIR}/valhalla
  PRIVATE
    ${CMAKE_SOURCE_DIR}/rapidjson/include)

# Common compile definitions
set(valhalla_compile_definitions "BOOST_NO_CXX11_SCOPED_ENUMS;RAPIDJSON_HAS_STDSTRING")
if (MSVC)
  list(APPEND valhalla_compile_definitions "VC_EXTRALEAN;WIN32_LEAN_AND_MEAN;NOMINMAX;NOGDI")
endif()

target_compile_definitions(valhalla PRIVATE ${valhalla_compile_definitions})


## Executable targets
function(get_source_path PATH NAME)
  if(EXISTS ${CMAKE_SOURCE_DIR}/src/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/${NAME}.cc PARENT_SCOPE)
  elseif(EXISTS ${CMAKE_SOURCE_DIR}/src/meili/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/meili/${NAME}.cc PARENT_SCOPE)
  elseif(EXISTS ${CMAKE_SOURCE_DIR}/src/mjolnir/${NAME}.cc)
    set(${PATH} ${CMAKE_SOURCE_DIR}/src/mjolnir/${NAME}.cc PARENT_SCOPE)
  else()
    message(ERROR "no source path for ${NAME}")
  endif()
endfunction()

## Valhalla programs
set(valhalla_programs valhalla_run_map_match valhalla_benchmark_loki valhalla_benchmark_skadi
  valhalla_run_isochrone valhalla_run_route valhalla_benchmark_adjacency_list valhalla_run_matrix
  valhalla_path_comparison valhalla_export_edges valhalla_pack_elevation)

## Valhalla data tools
set(valhalla_data_tools valhalla_ways_to_edges valhalla_validate_transit
  valhalla_benchmark_admins	valhalla_build_connectivity	valhalla_build_tiles
  valhalla_build_admins valhalla_build_transit valhalla_fetch_transit valhalla_query_transit
  valhalla_build_speeds  valhalla_associate_segments)

## Valhalla services
set(valhalla_services	valhalla_service valhalla_loki_worker	valhalla_odin_worker valhalla_thor_worker)

foreach(program ${valhalla_programs})
  get_source_path(path ${program})
  add_executable(${program} ${path})
  target_link_libraries(${program} Boost::program_options valhalla)
endforeach()

if(ENABLE_DATA_TOOLS)
  foreach(program ${valhalla_data_tools})
    get_source_path(path ${program})
    add_executable(${program} ${path})
    target_link_libraries(${program} valhalla GEOS::GEOS Boost::program_options)
  endforeach()

  add_executable(valhalla_build_statistics
    src/mjolnir/valhalla_build_statistics.cc
    src/mjolnir/statistics.cc
    src/mjolnir/statistics_database.cc)
  target_link_libraries(valhalla_build_statistics valhalla Boost::program_options)
endif()

if(ENABLE_SERVICES)
  foreach(program ${valhalla_services})
    add_executable(${program} ${CMAKE_SOURCE_DIR}/src/${program}.cc)
    target_link_libraries(${program} valhalla Boost::program_options)
  endforeach()
endif()


## Python bindings
if(ENABLE_PYTHON_BINDINGS)
  find_package(PythonInterp)
  find_package(PythonLibs)
  set(_Boost_PYTHON${PYTHON_VERSION_MAJOR}_HEADERS "boost/python.hpp")
  find_package(Boost COMPONENTS python${PYTHON_VERSION_MAJOR})
  python_add_module(python_valhalla src/bindings/python/python.cc)
  set_target_properties(python_valhalla PROPERTIES
    OUTPUT_NAME valhalla
    LINK_LIBRARIES ${PYTHON_LIBRARIES}
    INCLUDE_DIRECTORIES ${PYTHON_INCLUDE_DIRS})
  target_link_libraries(python_valhalla valhalla Boost::python${PYTHON_VERSION_MAJOR})
  set_target_properties(valhalla PROPERTIES
    POSITION_INDEPENDENT_CODE ON)
endif()


## Tests TODO: move to own namespace
## FAIL: astar narrativebuilder util_odin
set(tests aabb2 actor admin astar attributes_controller datetime directededge
  distanceapproximator double_bucket_queue edgecollapser edge_elevation edgestatus ellipse encode
  enhancedtrippath factory graphid graphreader graphtile gridded_data grid_range_query grid_traversal
  json laneconnectivity linesegment2 location logging maneuversbuilder map_matcher_factory
  narrativebuilder narrative_dictionary navigator nodeinfo obb2 optimizer  point2 pointll
  polyline2 queue routing sample sequence serializers sign signs streetname streetnames streetnames_factory
  streetnames_us streetname_us tilehierarchy tiles traffic_matcher  turn util_midgard
  util_odin util_skadi vector2 verbal_text_formatter verbal_text_formatter_us verbal_text_formatter_us_co
  verbal_text_formatter_us_tx viterbi_search)

if(ENABLE_DATA_TOOLS)
  list(APPEND tests mapmatch utrecht edgeinfobuilder uniquenames idtable graphbuilder graphparser
	  names refs signinfo countryaccess graphtilebuilder search node_search matrix)
endif()

if(ENABLE_SERVICES)
  list(APPEND tests skadi_service loki_service thor_service)
endif()

foreach(test ${tests})
  add_executable(${test} EXCLUDE_FROM_ALL	test/${test}.cc test/test.cc)
  set_target_properties(${test} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY unit_tests)
  target_link_libraries(${test} valhalla)
endforeach()

set_target_properties(logging PROPERTIES COMPILE_DEFINITIONS LOGGING_LEVEL_ALL)

add_custom_command(OUTPUT test/data/utrecht_tiles/0/003/196.gph
  COMMAND ${CMAKE_BINARY_DIR}/valhalla_build_tiles --inline-config '{"mjolnir":{"tile_dir":"test/data/utrecht_tiles","hierarchy":true,"shortcuts":true,"concurrency":4,"logging":{"type":""}}}' test/data/utrecht_netherlands.osm.pbf
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Building Utrecht Tiles..."
  DEPENDS valhalla_build_tiles)

set(cost_tests autocost bicyclecost motorscootercost pedestriancost transitcost truckcost)
foreach(test ${cost_tests})
  add_executable(${test} EXCLUDE_FROM_ALL	src/sif/${test}.cc test/test.cc)
  set_target_properties(${test} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY unit_tests
    COMPILE_DEFINITIONS INLINE_TEST)
  target_link_libraries(${test} valhalla)
  add_test(check-${test} COMMAND ${test})
endforeach()

add_custom_target(tests DEPENDS ${tests} ${cost_tests})

string(REGEX REPLACE "([^;]+)" "COMMAND;${CMAKE_BINARY_DIR}/unit_tests/\\1" test_commands "${tests};${cost_tests}")
add_custom_target(check ${test_commands}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  DEPENDS tests test/data/utrecht_tiles/0/003/196.gph)
