set(module meili)
set(library valhalla-${module})

set(headers
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
  ${CMAKE_SOURCE_DIR}/valhalla/meili/traffic_segment_matcher.h)

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

add_library(${library} STATIC ${sources} ${headers})
add_library(valhalla::${module} ALIAS ${library})
target_include_directories(${library}
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
  PRIVATE
    ${CMAKE_BINARY_DIR})

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${module})

if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
  set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
