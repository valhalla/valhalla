file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/tyr/*.h)

set(sources
  actor.cc
  height_serializer.cc
  isochrone_serializer.cc
  locate_serializer.cc
  matrix_serializer.cc
  route_serializer.cc
  serializers.cc
  trace_serializer.cc
  transit_available_serializer.cc)

set(sources_with_warnings
  route_serializer_osrm.cc
  route_serializer_valhalla.cc)

# treat date library as system
set(system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include)
if(APPLE)
  list(APPEND system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include/date)
endif()

valhalla_module(NAME tyr
  SOURCES
    ${sources}
  SOURCES_WITH_WARNINGS
    ${sources_with_warnings}
  HEADERS
    ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
      ${CMAKE_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
  DEPENDS
    valhalla::loki
    valhalla::thor
    valhalla::odin
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost)
