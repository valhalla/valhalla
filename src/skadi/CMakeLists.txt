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
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      $<$<BOOL:${WIN32}>:${dirent_include_dir}>
  DEPENDS
    valhalla::baldr
    Boost::boost
    PkgConfig::ZLIB
    PkgConfig::LZ4)
