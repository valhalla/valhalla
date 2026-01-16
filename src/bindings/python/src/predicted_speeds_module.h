#pragma once

#include <nanobind/nanobind.h>

namespace pyvalhalla {
void init_predicted_speeds(nanobind::module_& m);
}
