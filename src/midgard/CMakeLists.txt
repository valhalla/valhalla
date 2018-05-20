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

valhalla_module(NAME midgard
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}/valhalla)