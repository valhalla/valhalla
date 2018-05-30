file(GLOB headers ${VALHALLA_SOURCE_DIR}/valhalla/sif/*.h)

valhalla_module(NAME sif
  SOURCES
    autocost.cc
    bicyclecost.cc
    motorcyclecost.cc
    motorscootercost.cc
    pedestriancost.cc
    transitcost.cc
    truckcost.cc
    dynamiccost.cc
  HEADERS ${headers}
  INCLUDE_DIRECTORIES
    PUBLIC
      ${VALHALLA_SOURCE_DIR}
      ${VALHALLA_SOURCE_DIR}/valhalla
  DEPENDS
    valhalla::baldr)
