file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/loki/*.h)

set(sources
  worker.cc
  height_action.cc
  reach.cc
  matrix_action.cc
  status_action.cc
  transit_available_action.cc
  polygon_search.cc)

# Enables stricter compiler checks on a file-by-file basis
# which allows us to migrate piecemeal
set(sources_with_warnings
  isochrone_action.cc
  height_action.cc
  locate_action.cc
  node_search.cc
  route_action.cc
  search.cc
  trace_route_action.cc
  worker.cc)

set(system_includes 
  ${date_include_dir}
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>)

valhalla_module(NAME loki
  SOURCES ${sources}
  SOURCES_WITH_WARNINGS ${sources_with_warnings}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${rapidjson_include_dir}
      ${CMAKE_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
  DEPENDS
    valhalla::skadi
    valhalla::sif
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost
    ${libprime_server_targets})
