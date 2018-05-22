set(headers
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/linesegment2.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/tiles.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/gridded_data.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/polyline2.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/obb2.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/pointll.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/vector2.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/constants.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/aabb2.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/point2.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/util.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/distanceapproximator.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/ellipse.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/sequence.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/shape_decoder.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/encoded.h
  ${VALHALLA_SOURCE_DIR}/valhalla/midgard/logging.h)

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
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla)