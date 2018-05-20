valhalla_module(NAME skadi
  SOURCES
    sample.cc
    util.cc
  HEADERS
    ${CMAKE_SOURCE_DIR}/valhalla/skadi/sample.h
    ${CMAKE_SOURCE_DIR}/valhalla/skadi/util.h
  INCLUDE_DIRECTORIES
    PUBLIC
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}/valhalla
  DEPENDS
    valhalla::midgard
    Boost::filesystem
    Boost::regex
    LZ4::LZ4
    ZLIB::ZLIB)
