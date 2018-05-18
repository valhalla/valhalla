set(module skadi)
set(library valhalla-${module})

set(headers
  ${CMAKE_SOURCE_DIR}/valhalla/skadi/sample.h
  ${CMAKE_SOURCE_DIR}/valhalla/skadi/util.h)

set(sources
  sample.cc
  util.cc)

add_library(${library} STATIC ${sources} ${headers})
add_library(valhalla::${module} ALIAS ${library})
target_include_directories(${library}
  PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/valhalla)
  target_link_libraries(${library} valhalla::midgard Boost::filesystem Boost::regex LZ4::LZ4 ZLIB::ZLIB)

install(FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/valhalla/${module})

if(BUILD_SHARED_LIBS OR ENABLE_PYTHON_BINDINGS)
  set_property(TARGET ${library} PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()
