set(headers
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/costconstants.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/costfactory.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/autocost.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/bicyclecost.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/motorscootercost.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/pedestriancost.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/transitcost.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/truckcost.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/dynamiccost.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/hierarchylimits.h
  ${VALHALLA_SOURCE_DIR}/valhalla/sif/edgelabel.h)

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
  DEPENDS
    valhalla::baldr)
