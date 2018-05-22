set(headers
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/worker.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/astar.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/astarheuristic.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/bidirectional_astar.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/costmatrix.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/edgestatus.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/isochrone.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/optimizer.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/map_matcher.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/match_result.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/multimodal.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/pathalgorithm.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/pathinfo.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/route_matcher.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/trippathbuilder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/attributes_controller.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/trafficalgorithm.h
  ${VALHALLA_SOURCE_DIR}/valhalla/thor/timedistancematrix.h)

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
