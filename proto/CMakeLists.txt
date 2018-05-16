## Protobuf
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(protobuff_srcs protobuff_hdrs
  directions_options.proto
  navigator.proto
  route.proto
  tripcommon.proto
  tripdirections.proto
  trippath.proto

  fileformat.proto
  osmformat.proto
  segment.proto
  tile.proto
  transit.proto
  transit_fetch.proto)

if(NOT TARGET protobuf::libprotobuf)
  if(PROTOBUF_LIBRARY)
    add_library(protobuf::libprotobuf UNKNOWN IMPORTED)
    set_target_properties(protobuf::libprotobuf PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${PROTOBUF_INCLUDE_DIR}")
    if(EXISTS "${PROTOBUF_LIBRARY}")
      set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LOCATION "${PROTOBUF_LIBRARY}")
    endif()
    if(EXISTS "${PROTOBUF_LIBRARY_RELEASE}")
      set_property(TARGET protobuf::libprotobuf APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LOCATION_RELEASE "${PROTOBUF_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${PROTOBUF_LIBRARY_DEBUG}")
      set_property(TARGET protobuf::libprotobuf APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LOCATION_DEBUG "${PROTOBUF_LIBRARY_DEBUG}")
    endif()
  endif()
endif()

add_library(valhalla-protobuf STATIC ${protobuff_srcs} ${protobuff_hdrs})
add_library(valhalla::protobuf ALIAS valhalla-protobuf)
target_link_libraries(valhalla-protobuf protobuf::libprotobuf)

if(BUILD_SHARED_LIBS)
  set_property(TARGET valhalla-protobuf PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
