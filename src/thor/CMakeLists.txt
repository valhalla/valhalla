file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/thor/*.h)

set(sources
  astar.cc
  bidirectional_astar.cc
  costmatrix.cc
  isochrone.cc
  map_matcher.cc
  multimodal.cc
  optimizer.cc
  trippathbuilder.cc
  attributes_controller.cc
  route_matcher.cc
  timedep_forward.cc
  timedep_reverse.cc
  trafficalgorithm.cc
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
    PRIVATE
      ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::tyr
    valhalla::protobuf)
