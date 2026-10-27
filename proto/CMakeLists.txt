## Protobuf
set(protobuf_descriptors
 descriptors/api.proto
 descriptors/common.proto
 descriptors/directions.proto
 descriptors/info.proto
 descriptors/options.proto
 descriptors/sign.proto
 descriptors/trip.proto
 descriptors/transit.proto
 descriptors/transit_fetch.proto
 descriptors/incidents.proto
 descriptors/status.proto
 descriptors/matrix.proto
 descriptors/isochrone.proto
 descriptors/expansion.proto)

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
