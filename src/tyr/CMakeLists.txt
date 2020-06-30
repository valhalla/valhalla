file(GLOB headers ${valhalla_SOURCE_DIR}/valhalla/tyr/*.h)

valhalla_module(NAME tyr
  SOURCES
    serializers.cc
    isochrone_serializer.cc
    matrix_serializer.cc
    height_serializer.cc
    locate_serializer.cc
    route_serializer.cc
    route_serializer_valhalla.cc
    route_serializer_osrm.cc
    transit_available_serializer.cc
    trace_serializer.cc
    actor.cc
  HEADERS
    ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${valhalla_SOURCE_DIR}
      ${valhalla_SOURCE_DIR}/valhalla
      ${valhalla_SOURCE_DIR}/third_party/date/include
      $<$<BOOL:${MSVC}>:${valhalla_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${valhalla_SOURCE_DIR}/third_party/rapidjson/include
      ${CMAKE_BINARY_DIR}
  DEPENDS
    valhalla::loki
    valhalla::thor
    valhalla::odin
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost)
