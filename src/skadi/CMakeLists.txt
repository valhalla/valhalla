file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/skadi/*.h)

valhalla_module(NAME skadi
  SOURCES
    sample.cc
    util.cc
    ${VALHALLA_SOURCE_DIR}/third_party/lz4/lib/lz4.c
    ${VALHALLA_SOURCE_DIR}/third_party/lz4/lib/lz4hc.c
    ${VALHALLA_SOURCE_DIR}/third_party/lz4/lib/lz4frame.c
    ${VALHALLA_SOURCE_DIR}/third_party/lz4/lib/xxhash.c
  HEADERS
    ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/lz4/lib
  DEPENDS
    valhalla::baldr
    Boost::boost
    ZLIB::ZLIB)