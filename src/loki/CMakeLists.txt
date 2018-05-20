set(headers
  ${CMAKE_SOURCE_DIR}/valhalla/loki/worker.h
  ${CMAKE_SOURCE_DIR}/valhalla/loki/search.h
  ${CMAKE_SOURCE_DIR}/valhalla/loki/node_search.h)

set(sources
  search.cc
  worker.cc
  height_action.cc
  locate_action.cc
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
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
  PRIVATE
    ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::tyr
    valhalla::skadi
    valhalla::baldr
    valhalla::protobuf
    libprime_server)
