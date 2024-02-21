file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/thor/*.h)

set(sources
  astar_bss.cc
  alternates.cc
  bidirectional_astar.cc
  costmatrix.cc
  dijkstras.cc
  matrix_action.cc
  multimodal.cc
  route_action.cc
  timedistancebssmatrix.cc
  timedistancematrix.cc
  triplegbuilder.cc
  unidirectional_astar.cc
  worker.cc)

set(sources_with_warnings
  centroid.cc
  expansion_action.cc
  isochrone_action.cc
  isochrone.cc
  map_matcher.cc
  optimized_route_action.cc
  optimizer.cc
  route_matcher.cc
  status_action.cc
  trace_attributes_action.cc
  trace_route_action.cc
  triplegbuilder_utils.h)

set(system_includes
  ${date_include_dir}
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>)

valhalla_module(NAME thor
  SOURCES ${sources}
  SOURCES_WITH_WARNINGS ${sources_with_warnings}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
    PRIVATE
      ${rapidjson_include_dir}
      ${robinhoodhashing_include_dir}

  DEPENDS
    valhalla::proto
    valhalla::sif
    valhalla::meili
    ${valhalla_protobuf_targets}
    Boost::boost
    ${libprime_server_targets})