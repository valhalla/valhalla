## Protobuf
protobuf_generate_cpp(protobuff_srcs protobuff_hdrs
  directions.proto
  directions_options.proto
  tripcommon.proto
  tripdirections.proto
  trippath.proto
  fileformat.proto
  osmformat.proto
  transit.proto
  transit_fetch.proto)

valhalla_module(NAME proto
  SOURCES
    ${protobuff_srcs}
  HEADERS
    ${protobuff_hdrs}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS
    protobuf::libprotobuf-lite)
