#include "baldr/graphid.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

namespace py = pybind11;
namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

namespace {} // namespace

PYBIND11_MODULE(graph_utils, m) {
  py::class_<vb::GraphId>(m, "GraphId")
      .def(py::init())
      .def(py::init<uint32_t, uint32_t, uint32_t>(), py::arg("tileid"), py::arg("level"),
           py::arg("id"))
      .def(py::init<uint64_t>())
      .def(py::init<std::string>())
      .def_readonly("value", &vb::GraphId::value)
      .def("tileid", &vb::GraphId::tileid, "The tileid part of the GraphId")
      .def("level", &vb::GraphId::level, "The level part of the GraphId")
      .def("id", &vb::GraphId::id, "The object id part of the GraphId")
      .def("Is_Valid", &vb::GraphId::Is_Valid)
      .def("Tile_Base", &vb::GraphId::Tile_Base,
           "Construct a new GraphId with the object id portion omitted.")
      .def("tile_value", &vb::GraphId::tile_value,
           "Returns a value indicating the tile (level and tile id) of the graph Id.")
      .def(py::self + uint64_t())              // operator+(uint64_t)
      .def(py::self == py::self)               // operator==(const GraphId&)
      .def(py::self != py::self)               // operator!=(const GraphId&)
      .def("__bool__", &vb::GraphId::Is_Valid) // operator bool
      .def("__repr__", [](const vb::GraphId& graph_id) { return std::to_string(graph_id); });

  m.def(
      "get_tile_base_lon_lat",
      [](const vb::GraphId& graph_id) {
        const auto& tiles_at_level = vb::TileHierarchy::levels().at(graph_id.level());
        const auto pt = tiles_at_level.tiles.Base(graph_id.tileid());

        return py::make_tuple(pt.x(), pt.y());
      },
      py::arg("graph_id"), "Get the base lon/lat of this graphid's tile.");

  m.def(
      "get_tile_id_from_lon_lat",
      [](const uint8_t level, const py::tuple& coord) {
        if (py::len(coord) != 2) {
          throw py::cast_error("Invalid coordinate size, must be 2");
        }

        auto x = coord[0].cast<double>();
        auto y = coord[1].cast<double>();
        if ((x < -180. || x > 180.f) || (y < -90. || y > 90.)) {
          throw py::attribute_error("Invalid coordinate, remember it's (lon, lat)");
        }

        return vb::TileHierarchy::GetGraphId(vm::PointLL{x, y}, level);
      },
      py::arg("level"), py::arg("level"), "Get the base lon/lat of this graphid's tile.");

  m.def(
      "get_tile_ids_from_bbox",
      [](const double minx, const double miny, const double maxx, const double maxy,
         std::vector<uint32_t> levels) {
        if (!levels.size()) {
          levels.push_back(0);
          levels.push_back(1);
          levels.push_back(2);
        }

        // road levels + transit
        const uint32_t max_level = vb::TileHierarchy::levels().back().level + 1;

        const vm::AABB2<vm::PointLL> bbox{minx, miny, maxx, maxy};
        std::vector<vb::GraphId> tile_ids;
        for (const auto level : levels) {
          if (level >= max_level) {
            throw py::value_error("We only support " + std::to_string(max_level) +
                                  " hierarchy levels.");
          }
          const auto level_tile_ids = vb::TileHierarchy::levels().at(level).tiles.TileList(bbox);
          tile_ids.reserve(tile_ids.size() + level_tile_ids.size());
          for (const auto tid : level_tile_ids) {
            tile_ids.emplace_back(vb::GraphId{static_cast<uint32_t>(tid), level, 0});
          }
        }

        return tile_ids;
      },
      py::arg("minx"), py::arg("miny"), py::arg("maxx"), py::arg("maxy"),
      py::arg("levels") = std::vector<uint32_t>{},
      "Returns all tile_ids for the specified levels (default: all) which intersect the bbox");
}
