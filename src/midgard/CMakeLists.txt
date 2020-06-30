file(GLOB headers ${valhalla_SOURCE_DIR}/valhalla/midgard/*.h)

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
      ${valhalla_SOURCE_DIR}
      ${valhalla_SOURCE_DIR}/valhalla
      $<$<BOOL:${MSVC}>:${valhalla_SOURCE_DIR}/third_party/dirent/include>
  DEPENDS
    Boost::boost)
