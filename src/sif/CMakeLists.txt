set(module sif)
set(library valhalla-${module})

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

add_library(${library} STATIC ${sources} ${headers})
add_library(valhalla::${module} ALIAS ${library})
target_include_directories(${library}
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla)
target_link_libraries(${library} valhalla::baldr)

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${module})

if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
  set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
