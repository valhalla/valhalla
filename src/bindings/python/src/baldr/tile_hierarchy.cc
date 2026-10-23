#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"
#include "module.h"

#include <nanobind/nanobind.h>

namespace nb = nanobind;
namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;


namespace pyvalhalla::baldr {

void init_tilehierarchy(nb::module_& m) {
  nb::class_<vb::TileHierarchy>(
      m, "TileHierarchy",
      "Static hierarchy of routing tiles. Query one level at a time with "
      "GetGraphIds(bbox, level).")
      .def_static(
          "GetGraphId",
          [](const vm::PointLL& point, const uint32_t level) {
            if (level >= vb::TileHierarchy::levels().size()) throw nb::value_error("Invalid hierarchy level");
            return vb::TileHierarchy::GetGraphId(point, static_cast<uint8_t>(level));
          },
          nb::arg("point"), nb::arg("level"),
          "Get the tile GraphId containing the point at a single hierarchy level.\n\n"
          ":param point: The PointLL location within the tile.\n"
          ":param level: The hierarchy level.\n"
          ":returns: GraphId of the tile.\n"
          ":raises ValueError: When the level is invalid.")
      .def_static(
          "GetGraphIds",
          [](const vm::AABB2<vm::PointLL>& bbox, const uint32_t level) {
            if (level >= vb::TileHierarchy::levels().size()) throw nb::value_error("Invalid hierarchy level");
            return vb::TileHierarchy::GetGraphIds(bbox, static_cast<uint8_t>(level));
          },
          nb::arg("bbox"), nb::arg("level"),
          nb::call_guard<nb::gil_scoped_release>(),
          "Get all tile GraphIds intersecting the bbox at a single hierarchy level.\n\n"
          ":param bbox: The AABB2 bounding box of tiles to find.\n"
          ":param level: The hierarchy level.\n"
          ":returns: The list of tile GraphIds which intersect the bounding box.\n"
          ":raises ValueError: When the level is invalid.");
}

} // namespace pyvalhalla::baldr
