## Protobuf
set(protobuf_descriptors
  api.proto
  common.proto
  directions.proto
  info.proto
  options.proto
  sign.proto
  trip.proto
  transit.proto
  transit_fetch.proto
  incidents.proto
  status.proto
  matrix.proto
  isochrone.proto
  expansion.proto)

protobuf_generate_cpp(protobuf_srcs protobuf_hdrs ${protobuf_descriptors})

valhalla_module(NAME proto
  SOURCES
    ${protobuf_srcs}
  HEADERS
    ${protobuf_hdrs}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    ${valhalla_protobuf_targets})
