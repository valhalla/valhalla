#ifndef PYVALHALLA_MIDGARD_MODULE_H
#define PYVALHALLA_MIDGARD_MODULE_H
#include <nanobind/nanobind.h>
namespace pyvalhalla::midgard {
void init_pointll(nanobind::module_& m);
}
#endif
