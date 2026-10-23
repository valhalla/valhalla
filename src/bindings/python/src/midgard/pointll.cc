
#include "midgard/aabb2.h"
#include "midgard/pointll.h"

#include <nanobind/nanobind.h>
#include <new>
#include <string>
#include <nanobind/stl/string.h>

namespace nb = nanobind;
namespace vm = valhalla::midgard;

namespace pyvalhalla::midgard {

void init_pointll(nb::module_& m) {
  nb::class_<vm::PointLL>(
      m, "PointLL",
      "Longitude, latitude point in degrees (lon first, lat second).")
      .def(
          nb::init<double, double>(),
          nb::arg("lon"), nb::arg("lat"),
          "Constructs a PointLL from longitude and latitude in degrees.")
      .def("lng", &vm::PointLL::lng, "Gets the longitude in degrees.")
      .def("lat", &vm::PointLL::lat, "Gets the latitude in degrees.")
      .def("is_valid", &vm::PointLL::IsValid,
           "Returns false if the coordinates hold the invalid sentinel.")
      .def("__repr__", [](const vm::PointLL& p) {
        return nb::str(("<PointLL(" + std::to_string(p.lng()) + ", " +
                        std::to_string(p.lat()) + ")>").c_str());
      });

  nb::class_<vm::AABB2<vm::PointLL>>(
      m, "AABB2",
      "Axis-aligned bounding box of PointLL coordinates (lon/lat degrees).")
      .def(
          "__init__",
          [](vm::AABB2<vm::PointLL>* self, double minx, double miny,
             double maxx, double maxy) {
            new (self) vm::AABB2<vm::PointLL>{minx, miny, maxx, maxy};
          },
          nb::arg("minx"), nb::arg("miny"), nb::arg("maxx"), nb::arg("maxy"),
          "Constructs a bounding box from min/max lon/lat values.")
      .def(
          "__init__",
          [](vm::AABB2<vm::PointLL>* self, const vm::PointLL& minpt,
             const vm::PointLL& maxpt) {
            new (self) vm::AABB2<vm::PointLL>{minpt, maxpt};
          },
          nb::arg("minpt"), nb::arg("maxpt"),
          "Constructs a bounding box from minimum and maximum points.")
      .def("minx", &vm::AABB2<vm::PointLL>::minx, "Gets the minimum x (lon).")
      .def("maxx", &vm::AABB2<vm::PointLL>::maxx, "Gets the maximum x (lon).")
      .def("miny", &vm::AABB2<vm::PointLL>::miny, "Gets the minimum y (lat).")
      .def("maxy", &vm::AABB2<vm::PointLL>::maxy, "Gets the maximum y (lat).")
      .def("minpt", &vm::AABB2<vm::PointLL>::minpt, "Gets the minimum point.")
      .def("maxpt", &vm::AABB2<vm::PointLL>::maxpt, "Gets the maximum point.")
      .def("__repr__", [](const vm::AABB2<vm::PointLL>& b) {
        return nb::str(("<AABB2(" + std::to_string(b.minx()) + ", " +
                        std::to_string(b.miny()) + ", " +
                        std::to_string(b.maxx()) + ", " +
                        std::to_string(b.maxy()) + ")>").c_str());
      });

}

} // namespace pyvalhalla::midgard
