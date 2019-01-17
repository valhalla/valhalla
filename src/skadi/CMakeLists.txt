file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/skadi/*.h)

valhalla_module(NAME skadi
  SOURCES
    sample.cc
    util.cc
  HEADERS
    ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      $<$<BOOL:${MSVC}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
  DEPENDS
    valhalla::baldr
    Boost::boost
    ZLIB::ZLIB)
