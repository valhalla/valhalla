set(headers
  ${CMAKE_SOURCE_DIR}/valhalla/sif/costconstants.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/costfactory.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/autocost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/bicyclecost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/motorscootercost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/pedestriancost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/transitcost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/truckcost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/dynamiccost.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/hierarchylimits.h
  ${CMAKE_SOURCE_DIR}/valhalla/sif/edgelabel.h)

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
      ${CMAKE_SOURCE_DIR}
      ${CMAKE_SOURCE_DIR}/valhalla
  DEPENDS
    valhalla::baldr)
