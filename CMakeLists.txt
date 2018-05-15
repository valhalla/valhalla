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
include_directories(${CURL_INCLUDE_DIR})

find_package(LZ4 REQUIRED)
include_directories(${LZ4_INCLUDE_DIR})

find_package(Lua REQUIRED)
include_directories(${LUA_INCLUDE_DIR})

find_package(SQLite3 REQUIRED)

find_package(SpatiaLite)

find_package(ZLIB REQUIRED)

if(ENABLE_CCACHE AND (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
  find_program(CCACHE_FOUND ccache)
  if(CCACHE_FOUND)
    message(STATUS "Using ccache to speed up incremental builds")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    set(ENV{CCACHE_CPP2} "true")
  endif()
endif()

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


find_program(XXD xxd)
if (NOT XXD)
  message(FATAL_ERROR "xxd executable not found")
endif()

add_custom_command(OUTPUT date_time_zonespec.h
  COMMAND ${XXD} -i -s `head -1 date_time/zonespec.csv | wc -c` date_time/zonespec.csv ${CMAKE_BINARY_DIR}/date_time_zonespec.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling date_time/zonespec.csv to date_time_zonespec.h"
  DEPENDS date_time/zonespec.csv)

message(STATUS "Configuring lua/graph.lua to generate graph_lua_proc.h")
add_custom_command(OUTPUT graph_lua_proc.h
  COMMAND ${XXD} -i lua/graph.lua ${CMAKE_BINARY_DIR}/graph_lua_proc.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling lua/graph.lua to graph_lua_proc.h"
  DEPENDS lua/graph.lua
  VERBATIM)

message(STATUS "Configuring lua/admin.lua to generate admin_lua_proc.h")
add_custom_command(OUTPUT admin_lua_proc.h
  COMMAND ${XXD} -i lua/admin.lua ${CMAKE_BINARY_DIR}/admin_lua_proc.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Compiling lua/admin.lua to admin_lua_proc.h"
  DEPENDS lua/admin.lua
  VERBATIM)

message(STATUS "Configuring locales/*.json to generate locales.h")
file(GLOB locale_jsons ${CMAKE_SOURCE_DIR}/locales *.json)
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/locales.h
  COMMAND echo "#include <unordered_map>\\n" > ${CMAKE_BINARY_DIR}/locales.h
  COMMAND find . -name *.json -exec ${XXD} -i {} \; >> ${CMAKE_BINARY_DIR}/locales.h
  COMMAND echo "const std::unordered_map<std::string, std::string> locales_json = {" >> ${CMAKE_BINARY_DIR}/locales.h
  COMMAND find . -name *.json -printf "%f\\n" | sed s/.json// | sed s@[./-]@_@g  | awk "{ printf \"    {\\\"%s\\\", {__%s_json, __%s_json + __%s_json_len}},\\n\", $1, $1, $1, $1 }"  >> ${CMAKE_BINARY_DIR}/locales.h
  COMMAND echo "};\\n" >> ${CMAKE_BINARY_DIR}/locales.h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/locales
  COMMENT "Compiling lua/admin.lua to admin_lua_proc.h"
  DEPENDS ${locale_jsons}
  VERBATIM)

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

message(STATUS "Generating  ${CMAKE_BINARY_DIR}/valhalla/valhalla.h")
configure_file(${CMAKE_SOURCE_DIR}/valhalla/valhalla.h.in ${CMAKE_BINARY_DIR}/valhalla/valhalla.h @ONLY)

message(STATUS "Generating  ${CMAKE_BINARY_DIR}/valhalla/config.h")
configure_file(${CMAKE_SOURCE_DIR}/valhalla/config.h.cmake config.h @ONLY)

# Common includes
set(_include_directories
  ${CMAKE_BINARY_DIR}
  ${CMAKE_BINARY_DIR}/valhalla
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/valhalla
  ${SQLITE3_INCLUDE_DIR})
if (SPATIALITE_FOUND)
  set(MAYBE_SPATIALITE_LIBRARY ${SPATIALITE_LIBRARY})
  list(APPEND _include_directories ${SPATIALITE_INCLUDE_DIR})
endif()

# Common compile definitions
set(_compile_definitions)
if (MSVC)
  list(APPEND _compile_definitions "VC_EXTRALEAN;WIN32_LEAN_AND_MEAN;NOMINMAX;NOGDI")
endif()
list(APPEND _compile_definitions "BOOST_NO_CXX11_SCOPED_ENUMS")
list(APPEND _compile_definitions "BOOST_SPIRIT_THREADSAFE")
list(APPEND _compile_definitions "CURL_STATICLIB")
if (SPATIALITE_FOUND)
  list(APPEND _compile_definitions "HAVE_SPATIALITE")
endif()

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
  ${CMAKE_BINARY_DIR}/date_time_zonespec.h
  ${CMAKE_BINARY_DIR}/locales.h)

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
  ${CMAKE_BINARY_DIR}/graph_lua_proc.h
  ${CMAKE_BINARY_DIR}/admin_lua_proc.h)

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
  Boost::date_time
  Boost::filesystem
  Boost::iostreams
  Boost::program_options
  Boost::regex
  Boost::system
  Boost::thread
  ${CURL_LIBRARIES})

target_include_directories(valhalla PRIVATE
  ${_include_directories}
  ${CMAKE_SOURCE_DIR}/rapidjson/include)

set(valhalla_compile_definitions ${_compile_definitions})
list(APPEND valhalla_compile_definitions "RAPIDJSON_HAS_STDSTRING")
target_compile_definitions(valhalla PRIVATE ${valhalla_compile_definitions})

# valhalla data tools

set(valhalla_data_tools valhalla_build_tiles)
foreach(program ${valhalla_data_tools})
  message(STATUS "Configuring ${program} executable target")
  add_executable(${program} ${CMAKE_SOURCE_DIR}/src/mjolnir/${program}.cc)
  target_include_directories(${program} PRIVATE ${_include_directories})
  target_compile_definitions(${program} PRIVATE ${_compile_definitions})
  target_link_libraries(${program}
    valhalla
    ${LUA_LIBRARIES}
    ${LZ4_LIBRARY}
    ${MAYBE_SPATIALITE_LIBRARY}
    ${SQLITE3_LIBRARY}
    ZLIB::ZLIB)
endforeach()

# valhalla programs

set(valhalla_programs valhalla_run_route valhalla_run_isochrone)
foreach(program ${valhalla_programs})
  message(STATUS "Configuring ${program} executable target")
  add_executable(${program} ${CMAKE_SOURCE_DIR}/src/${program}.cc)
  target_include_directories(${program} PRIVATE ${_include_directories})
  target_compile_definitions(${program} PRIVATE ${_compile_definitions})
  target_link_libraries(${program} valhalla)
endforeach()


add_executable(valhalla_service ${CMAKE_SOURCE_DIR}/src/valhalla_service.cc)
target_include_directories(valhalla_service PRIVATE ${_include_directories})
target_compile_definitions(valhalla_service PRIVATE ${_compile_definitions})
target_link_libraries(valhalla_service valhalla ${LZ4_LIBRARY} ZLIB::ZLIB)
