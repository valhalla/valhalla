#include "graph_utils_module.h"

#include <nanobind/nanobind.h>

NB_MODULE(graph_utils, m) {
  pyvalhalla::init_graphid(m);
}
