#ifndef PYVALHALLA_BALDR_UTILS_H
#define PYVALHALLA_BALDR_UTILS_H

#include <nanobind/nanobind.h>

namespace pyvalhalla::baldr::utils {
void init_graphreader(nanobind::module_& m);
void init_graphtile(nanobind::module_& m);
void init_predicted_speeds(nanobind::module_& m);
} // namespace pyvalhalla::baldr::utils

#endif // PYVALHALLA_BALDR_UTILS_H
