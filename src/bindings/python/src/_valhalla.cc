#include "baldr/module.h"
#include "baldr/utils/module.h"
#include "config.h"
#include "module.h"

#include <nanobind/nanobind.h>

NB_MODULE(_valhalla, m) {
  // Add this constant in C++ to avoid creating another shim python module, as it
  // is needed at runtime of the python library and must be set during the build.
  m.attr("VALHALLA_PRINT_VERSION") = VALHALLA_PRINT_VERSION;

  pyvalhalla::init_exceptions(m);
  pyvalhalla::init_actor(m);
  pyvalhalla::baldr::init_graphid(m);
  pyvalhalla::baldr::utils::init_graphreader(m);
  pyvalhalla::baldr::utils::init_graphtile(m);
  pyvalhalla::baldr::utils::init_predicted_speeds(m);
}
