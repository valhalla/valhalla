#include "predicted_speeds_module.h"

#include <nanobind/nanobind.h>

NB_MODULE(predicted_speeds, m) {
  pyvalhalla::init_predicted_speeds(m);
  m.doc() = "Valhalla DCT-2 speed compression utilities";
}
