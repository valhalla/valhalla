set(headers
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/universal_cost.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/candidate_search.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/geometry_helpers.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/grid_range_query.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/emission_cost_model.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/transition_cost_model.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/priority_queue.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/routing.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/stateid.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/viterbi_search.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/topk_search.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/measurement.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/match_result.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/grid_traversal.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/state.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/map_matcher.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/map_matcher_factory.h
  ${VALHALLA_SOURCE_DIR}/valhalla/meili/traffic_segment_matcher.h)

set(sources
  universal_cost.cc
  viterbi_search.cc
  topk_search.cc
  routing.cc
  candidate_search.cc
  transition_cost_model.cc
  map_matcher.cc
  map_matcher_factory.cc
  match_route.cc
  traffic_segment_matcher.cc)

valhalla_module(NAME meili
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::baldr)
