valhalla_module(NAME skadi
  SOURCES
    sample.cc
    util.cc
  HEADERS
    ${VALHALLA_SOURCE_DIR}/valhalla/skadi/sample.h
    ${VALHALLA_SOURCE_DIR}/valhalla/skadi/util.h
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
  DEPENDS
    valhalla::midgard
    Boost::filesystem
    Boost::regex
    LZ4::LZ4
    ZLIB::ZLIB)
