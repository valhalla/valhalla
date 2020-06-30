file(GLOB headers ${valhalla_SOURCE_DIR}/valhalla/loki/*.h)

set(sources
  search.cc
  worker.cc
  height_action.cc
  locate_action.cc
  reach.cc
  route_action.cc
  matrix_action.cc
  isochrone_action.cc
  trace_route_action.cc
  transit_available_action.cc
  node_search.cc)

valhalla_module(NAME loki
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${valhalla_SOURCE_DIR}
      ${valhalla_SOURCE_DIR}/valhalla
      ${valhalla_SOURCE_DIR}/third_party/date/include
      $<$<BOOL:${MSVC}>:${valhalla_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${valhalla_SOURCE_DIR}/third_party/rapidjson/include
      ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::skadi
    valhalla::sif
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost
    libprime_server)
