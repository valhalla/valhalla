set(module thor)
set(library valhalla-${module})

set(headers
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
  ${CMAKE_SOURCE_DIR}/valhalla/thor/timedistancematrix.h)

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

add_library(${library} STATIC ${sources} ${headers})
add_library(valhalla::${module} ALIAS ${library})
target_compile_definitions(${library} PUBLIC RAPIDJSON_HAS_STDSTRING)
target_include_directories(${library}
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
  PRIVATE
    ${CMAKE_BINARY_DIR})
target_link_libraries(${library} valhalla::tyr valhalla::protobuf)

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${module})

if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
  set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
