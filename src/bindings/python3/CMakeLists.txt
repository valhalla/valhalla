add_subdirectory(../../../third_party/pybind11 pybind11_build)

pybind11_add_module(example src/python_pybind.cc)
target_link_libraries(example PUBLIC valhalla)
configure_file(valhalla/__init__.py.in valhalla/__init__.py)
