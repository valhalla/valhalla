#ifndef PYVALHALLA_H
#define PYVALHALLA_H

#include <nanobind/nanobind.h>

namespace pyvalhalla {
void init_exceptions(nanobind::module_& m);
void init_actor(nanobind::module_& m);
} // namespace pyvalhalla

#endif // PYVALHALLA_H
