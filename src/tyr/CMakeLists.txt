set(module tyr)
set(library valhalla-${module})

set(headers
  ${CMAKE_SOURCE_DIR}/valhalla/tyr/serializers.h
  ${CMAKE_SOURCE_DIR}/valhalla/tyr/navigator.h
  ${CMAKE_SOURCE_DIR}/valhalla/tyr/actor.h)

set(sources
  ${CMAKE_SOURCE_DIR}/src/worker.cc
  serializers.cc
  isochrone_serializer.cc
  matrix_serializer.cc
  height_serializer.cc
  locate_serializer.cc
  route_serializer.cc
  transit_available_serializer.cc
  trace_serializer.cc
  navigator.cc
  actor.cc)

add_library(${library} STATIC ${sources} ${headers})
add_library(valhalla::${module} ALIAS ${library})
target_include_directories(${library}
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla
  PRIVATE
    ${CMAKE_BINARY_DIR})
target_link_libraries(${library} valhalla::loki valhalla::thor valhalla::odin LZ4::LZ4 ZLIB::ZLIB)

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${module})

if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
  set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
