if(ENABLE_DATA_TOOLS)
    add_library(valhalla_datatools_test STATIC
      test.h test.cc
      ${VALHALLA_SOURCE_DIR}/third_party/microtar/src/microtar.h
      ${VALHALLA_SOURCE_DIR}/third_party/microtar/src/microtar.c)
    target_link_libraries(valhalla_datatools_test valhalla_test)
    target_include_directories(valhalla_test PUBLIC ${VALHALLA_SOURCE_DIR}/third_party/microtar/src)
endif()
