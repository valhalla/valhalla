file(GLOB headers ${valhalla_SOURCE_DIR}/valhalla/sif/*.h)

set(sources
  autocost.cc
  bicyclecost.cc
  hierarchylimits.cc
  motorcyclecost.cc
  motorscootercost.cc
  nocost.cc
  pedestriancost.cc
  transitcost.cc
  truckcost.cc
  dynamiccost.cc)

valhalla_module(NAME sif
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${valhalla_SOURCE_DIR}
      ${valhalla_SOURCE_DIR}/valhalla
      ${valhalla_SOURCE_DIR}/third_party/date/include
      $<$<BOOL:${MSVC}>:${valhalla_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${valhalla_SOURCE_DIR}/third_party/rapidjson/include
  DEPENDS
    valhalla::baldr
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost)
