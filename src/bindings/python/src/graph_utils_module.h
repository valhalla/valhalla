#ifndef PYVALHALLA_GRAPH_UTILS_MODULE_H
#define PYVALHALLA_GRAPH_UTILS_MODULE_H

#include <pybind11/pybind11.h>

namespace pyvalhalla {
void init_graphid(pybind11::module& m);
} // namespace pyvalhalla

#endif // PYVALHALLA_GRAPH_UTILS_MODULE_H
