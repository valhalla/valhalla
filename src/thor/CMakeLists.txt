file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/thor/*.h)

set(sources
    alternates.cc
    triplegbuilder.cc
    worker.cc)

set(sources_with_warnings
  astar_bss.cc
  bidirectional_astar.cc
  centroid.cc
  costmatrix.cc
  dijkstras.cc
  expansion_action.cc
  isochrone_action.cc
  isochrone.cc
  map_matcher.cc
  matrix_action.cc
  multimodal.cc
  optimized_route_action.cc
  optimizer.cc
  route_action.cc
  route_matcher.cc
  status_action.cc
  timedistancematrix.cc
  timedistancebssmatrix.cc
  trace_attributes_action.cc
  trace_route_action.cc
  triplegbuilder_utils.h
  unidirectional_astar.cc)

# treat date library as system
set(system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include)
if(APPLE)
  list(APPEND system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include/date)
endif()

valhalla_module(NAME thor
  SOURCES ${sources}
  SOURCES_WITH_WARNINGS ${sources_with_warnings}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
      ${CMAKE_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
  DEPENDS
    valhalla::proto
    valhalla::sif
    valhalla::meili
    ${valhalla_protobuf_targets}
    Boost::boost
    libprime_server
    robin_hood::robin_hood)
