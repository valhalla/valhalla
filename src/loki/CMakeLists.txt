set(module loki)
set(library valhalla-${module})

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

add_library(${library} STATIC ${sources} ${headers})
add_library(valhalla::${module} ALIAS ${library})
target_compile_definitions(${library} PUBLIC RAPIDJSON_HAS_STDSTRING)
target_include_directories(${library}
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
  PRIVATE
    ${CMAKE_BINARY_DIR})
target_link_libraries(${library} valhalla::tyr valhalla::skadi valhalla::baldr valhalla::protobuf libprime_server)

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${module})

if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
  set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
