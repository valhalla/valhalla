#ifndef PYVALHALLA_MIDGARD_UTILS_H
#define PYVALHALLA_MIDGARD_UTILS_H

#include <nanobind/nanobind.h>

namespace pyvalhalla::midgard::utils {
void init_polyline(nanobind::module_& m);
} // namespace pyvalhalla::midgard::utils

#endif // PYVALHALLA_MIDGARD_UTILS_H
