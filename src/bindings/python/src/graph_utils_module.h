#ifndef PYVALHALLA_GRAPH_UTILS_MODULE_H
#define PYVALHALLA_GRAPH_UTILS_MODULE_H

#include <nanobind/nanobind.h>

namespace pyvalhalla {
void init_graphid(nanobind::module_& m);
} // namespace pyvalhalla

#endif // PYVALHALLA_GRAPH_UTILS_MODULE_H
