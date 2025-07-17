#include "graph_utils_module.h"

#include <pybind11/pybind11.h>

PYBIND11_MODULE(graph_utils, m) {
  pyvalhalla::init_graphid(m);
}
