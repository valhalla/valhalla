#include "graph_utils_module.h"

#include <pybind11/detail/common.h>

PYBIND11_MODULE(graph_utils, m) {
  pyvalhalla::init_graphid(m);
}
