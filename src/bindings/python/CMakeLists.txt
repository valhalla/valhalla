add_subdirectory(${VALHALLA_SOURCE_DIR}/third_party/pybind11 ${CMAKE_BINARY_DIR}/third_party/pybind11)

pybind11_add_module(python_valhalla python.cc)
target_link_libraries(python_valhalla PUBLIC valhalla)
set_target_properties(python_valhalla PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla)
set_target_properties(valhalla PROPERTIES POSITION_INDEPENDENT_CODE ON)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py ${CMAKE_CURRENT_BINARY_DIR}/valhalla/__init__.py COPYONLY)
configure_file(${VALHALLA_SOURCE_DIR}/scripts/valhalla_build_config ${CMAKE_CURRENT_BINARY_DIR}/valhalla/valhalla_build_config.py COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.py ${CMAKE_CURRENT_BINARY_DIR}/valhalla/config.py COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/actor.py ${CMAKE_CURRENT_BINARY_DIR}/valhalla/actor.py COPYONLY)

message(STATUS "Installing python modules to ${Python_SITEARCH}")
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/valhalla
DESTINATION "${Python_SITEARCH}"
COMPONENT python_valhalla
FILES_MATCHING PATTERN "*.py")
install(TARGETS python_valhalla
DESTINATION "${Python_SITEARCH}/valhalla"
COMPONENT python_valhalla)
