file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/sif/*.h)

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
  dynamiccost.cc
  recost.cc)

# treat date library as system
set(system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include)
if(APPLE)
  list(APPEND system_includes ${VALHALLA_SOURCE_DIR}/third_party/date/include/date)
endif()

valhalla_module(NAME sif
  SOURCES ${sources}
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
      $<$<BOOL:${WIN32}>:${VALHALLA_SOURCE_DIR}/third_party/dirent/include>
    PRIVATE
      ${VALHALLA_SOURCE_DIR}/third_party/rapidjson/include
  SYSTEM_INCLUDE_DIRECTORIES
    PUBLIC
      ${system_includes}
  DEPENDS
    valhalla::baldr
    valhalla::proto
    ${valhalla_protobuf_targets}
    Boost::boost)
