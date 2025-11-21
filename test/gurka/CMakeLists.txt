if(ENABLE_DATA_TOOLS)
  file(GLOB_RECURSE TEST_FILES "${CMAKE_CURRENT_LIST_DIR}/test_*.cc")

  if (UNIX AND ENABLE_SINGLE_FILES_WERROR)
    set_source_files_properties(${TEST_FILES} PROPERTIES COMPILE_FLAGS "-Wall -Werror")
  endif()

  add_executable(gurka EXCLUDE_FROM_ALL ${TEST_FILES})
  set_target_properties(gurka PROPERTIES FOLDER "Tests")
  create_source_groups("Source Files" ${TEST_FILES})
  target_link_libraries(gurka valhalla_test)
  if (LuaJIT_FOUND AND APPLE AND CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
    # Using LuaJIT on macOS on Intel processors requires a couple of extra linker flags
    target_link_options(gurka PUBLIC -pagezero_size 10000 -image_base 100000000)
  endif()
  add_dependencies(gurka build_timezones)

  ## Test run target
  add_custom_command(OUTPUT gurka.log
    COMMAND ${CMAKE_COMMAND} -E env LOCPATH=${VALHALLA_SOURCE_DIR}/locales ${VALHALLA_SOURCE_DIR}/scripts/run_single_test.sh ${CMAKE_CURRENT_BINARY_DIR}/gurka
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS gurka
    VERBATIM)
  add_custom_target(run-gurka DEPENDS gurka.log)
  set_target_properties(run-gurka PROPERTIES FOLDER "Tests")

  add_dependencies(tests gurka)
  add_dependencies(check run-gurka)
endif()
