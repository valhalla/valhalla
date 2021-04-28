file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/midgard/*.h)

if ((UNIX OR APPLE) AND ENABLE_SINGLE_FILES_WERROR)
 set_source_files_properties(
        polyline2.cc
        point_tile_index.cc
        PROPERTIES COMPILE_FLAGS "-Werror -Wuninitialized")
endif()

set(sources
  linesegment2.cc
  tiles.cc
  polyline2.cc
  obb2.cc
  pointll.cc
  point_tile_index.cc
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
      ${VALHALLA_SOURCE_DIR}/valhalla
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
  DEPENDS
    Boost::boost)
