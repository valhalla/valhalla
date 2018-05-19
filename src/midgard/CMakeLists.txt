set(module midgard)
set(library valhalla-${module})

set(headers
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/linesegment2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/tiles.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/gridded_data.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/polyline2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/obb2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/pointll.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/vector2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/constants.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/aabb2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/point2.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/util.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/distanceapproximator.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/ellipse.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/sequence.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/shape_decoder.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/encoded.h
  ${CMAKE_SOURCE_DIR}/valhalla/midgard/logging.h)

set(sources
  linesegment2.cc
  tiles.cc
  gridded_data.cc
  polyline2.cc
  obb2.cc
  pointll.cc
  aabb2.cc
  point2.cc
  util.cc
  ellipse.cc
  logging.cc)

add_library(${library} STATIC ${sources} ${headers})
add_library(valhalla::${module} ALIAS ${library})
target_include_directories(${library}
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla)

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${module} COMPONENT development)

if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
  set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
