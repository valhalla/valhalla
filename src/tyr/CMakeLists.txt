file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/tyr/*.h)

valhalla_module(NAME tyr
  SOURCES
    ${VALHALLA_SOURCE_DIR}/src/worker.cc
    serializers.cc
    isochrone_serializer.cc
    matrix_serializer.cc
    height_serializer.cc
    locate_serializer.cc
    route_serializer.cc
    transit_available_serializer.cc
    trace_serializer.cc
    navigator.cc
    actor.cc
  HEADERS
    ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
    PRIVATE
      ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::loki
    valhalla::thor
    valhalla::odin
    LZ4::LZ4
    ZLIB::ZLIB)
