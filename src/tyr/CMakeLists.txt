file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/tyr/*.h)

set(sources
  actor.cc
  height_serializer.cc
  isochrone_serializer.cc
  matrix_serializer.cc
  route_serializer_osrm.cc
  route_summary_cache.cc
  serializers.cc
  transit_available_serializer.cc
  expansion_serializer.cc)

set(sources_with_warnings
  locate_serializer.cc
  route_serializer.cc
  route_serializer_valhalla.cc
  trace_serializer.cc)

set(system_includes
  ${date_include_dir}
  $<$<BOOL:${WIN32}>:${dirent_include_dir}>)

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
    PRIVATE
      ${CMAKE_BINARY_DIR}
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
    PRIVATE
      ${rapidjson_include_dir}
  DEPENDS
    valhalla::loki
    valhalla::thor
    valhalla::odin
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost
    ${GDAL_TARGET}
    )
