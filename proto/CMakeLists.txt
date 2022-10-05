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
  status.proto)

if(ENABLE_DATA_TOOLS)
  # Only mjolnir needs the OSM PBF descriptors
  list(APPEND protobuf_descriptors
    ${VALHALLA_SOURCE_DIR}/third_party/OSM-binary/src/fileformat.proto
    ${VALHALLA_SOURCE_DIR}/third_party/OSM-binary/src/osmformat.proto)
endif()

protobuf_generate_cpp(protobuff_srcs protobuff_hdrs ${protobuf_descriptors})

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
