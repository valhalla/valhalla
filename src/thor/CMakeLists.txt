file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/thor/*.h)

set(sources
  astar.cc
  bidirectional_astar.cc
  costmatrix.cc
  isochrone.cc
  map_matcher.cc
  multimodal.cc
  optimizer.cc
  triplegbuilder.cc
  attributes_controller.cc
  route_matcher.cc
  timedep_forward.cc
  timedep_reverse.cc
  timedistancematrix.cc
  worker.cc
  isochrone_action.cc
  matrix_action.cc
  optimized_route_action.cc
  route_action.cc
  trace_attributes_action.cc
  trace_route_action.cc)

valhalla_module(NAME thor
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      ${VALHALLA_SOURCE_DIR}/third_party/date/include
      $<$<BOOL:${MSVC}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
      ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::proto
    valhalla::sif
    valhalla::meili
    ${valhalla_protobuf_targets}
    Boost::boost
    libprime_server)
