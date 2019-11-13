file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/sif/*.h)

set(sources
  autocost.cc
  bicyclecost.cc
  motorcyclecost.cc
  motorscootercost.cc
  pedestriancost.cc
  transitcost.cc
  truckcost.cc
  dynamiccost.cc)

valhalla_module(NAME sif
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      ${VALHALLA_SOURCE_DIR}/third_party/date/include
      $<$<BOOL:${MSVC}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
  DEPENDS
    valhalla::baldr
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost)
