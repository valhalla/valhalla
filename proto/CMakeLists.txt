## Protobuf
protobuf_generate_cpp(protobuff_srcs protobuff_hdrs
  api.proto
  directions.proto
  info.proto
  options.proto
  sign.proto
  tripcommon.proto
  trip.proto
  transit.proto
  transit_fetch.proto
  incidents.proto
  status.proto
  ${VALHALLA_SOURCE_DIR}/third_party/OSM-binary/src/fileformat.proto
  ${VALHALLA_SOURCE_DIR}/third_party/OSM-binary/src/osmformat.proto)

valhalla_module(NAME proto
  SOURCES
    ${protobuff_srcs}
  HEADERS
    ${protobuff_hdrs}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${valhalla_protobuf_targets})
