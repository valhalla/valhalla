file(GLOB headers ${valhalla_SOURCE_DIR}/valhalla/skadi/*.h)

valhalla_module(NAME skadi
  SOURCES
    sample.cc
    util.cc
  HEADERS
    ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${valhalla_SOURCE_DIR}
      ${valhalla_SOURCE_DIR}/valhalla
      $<$<BOOL:${MSVC}>:${valhalla_SOURCE_DIR}/third_party/dirent/include>
  DEPENDS
    valhalla::baldr
    Boost::boost
    ZLIB::ZLIB)
