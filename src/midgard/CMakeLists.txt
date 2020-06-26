file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/midgard/*.h)

set(sources
  tiles.cc
  gridded_data.cc
  polyline2.cc
  obb2.cc
  aabb2.cc
  util.cc
  ellipse.cc
  logging.cc)

valhalla_module(NAME midgard
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      $<$<BOOL:${MSVC}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>)
