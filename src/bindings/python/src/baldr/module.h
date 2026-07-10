#ifndef PYVALHALLA_BALDR_H
#define PYVALHALLA_BALDR_H

#include <nanobind/nanobind.h>

namespace pyvalhalla::baldr {
void init_graphid(nanobind::module_& m);
void init_graphtileheader(nanobind::module_& m);
} // namespace pyvalhalla::baldr

#endif // PYVALHALLA_BALDR_H
