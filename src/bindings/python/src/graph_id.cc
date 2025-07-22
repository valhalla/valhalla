#include "baldr/graphid.h"
#include "baldr/tilehierarchy.h"
#include "graph_utils_module.h"
#include "midgard/aabb2.h"
#include "midgard/pointll.h"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

namespace pyvalhalla {

// road levels + transit
const uint32_t MAX_LEVEL = vb::TileHierarchy::levels().back().level + 1;

auto check_level = [](const uint32_t level) {
  if (level >= MAX_LEVEL) {
    throw py::value_error("We only support " + std::to_string(MAX_LEVEL) + " hierarchy levels.");
  }
};

auto check_coord = [](const double minx, const double miny, const double maxx, const double maxy) {
  if (minx < -180. || maxx > 180. || miny < -90. || maxy > 90.) {
    throw py::value_error("Invalid coordinate, remember it's (lon, lat)");
  }
};

void init_graphid(pybind11::module& m) {
  py::class_<vb::GraphId>(m, "GraphId")
      .def(py::init())
      .def(py::init<uint32_t, uint32_t, uint32_t>(), py::arg("tileid"), py::arg("level"),
           py::arg("id"))
      .def(py::init<uint64_t>())
      .def(py::init<std::string>())
      .def_readonly("value", &vb::GraphId::value)
      .def("tileid", &vb::GraphId::tileid)
      .def("level", &vb::GraphId::level)
      .def("id", &vb::GraphId::id)
      .def("Is_Valid", &vb::GraphId::Is_Valid)
      .def("Tile_Base", &vb::GraphId::Tile_Base)
      .def("tile_value", &vb::GraphId::tile_value)
      .def(py::self + uint64_t())              // operator+(uint64_t)
      .def(py::self += uint32_t())             // operator+=(uint32_t)
      .def(py::self == py::self)               // operator==(const GraphId&)
      .def(py::self != py::self)               // operator!=(const GraphId&)
      .def("__bool__", &vb::GraphId::Is_Valid) // operator bool
      .def("__repr__",
           [](const vb::GraphId& graph_id) { return "<GraphId(" + std::to_string(graph_id) + ")>"; })
      // pickling support auto-provides copy/deepcopy support
      .def(py::pickle(
          // get_state
          [](const vb::GraphId& gid) { return py::make_tuple(gid.value); },
          // set_state
          [](py::tuple t) {
            vb::GraphId gid(t[0].cast<uint64_t>());
            return gid;
          }));

  m.def(
      "get_tile_base_lon_lat",
      [](const vb::GraphId& graph_id) {
        const auto& tiles_at_level = vb::TileHierarchy::levels().at(graph_id.level());
        const auto pt = tiles_at_level.tiles.Base(graph_id.tileid());

        return py::make_tuple(pt.x(), pt.y());
      },
      py::arg("graph_id"));

  m.def(
      "get_tile_id_from_lon_lat",
      [](const uint32_t level, const py::tuple& coord) {
        if (py::len(coord) != 2) {
          throw py::value_error("Invalid coordinate size, must be 2");
        }
        check_level(level);

        auto x = coord[0].cast<double>();
        auto y = coord[1].cast<double>();
        check_coord(x, y, x, y);

        return vb::TileHierarchy::GetGraphId(vm::PointLL{x, y}, static_cast<uint8_t>(level));
      },
      py::arg("level"), py::arg("coord"));

  m.def(
      "get_tile_ids_from_bbox",
      [](const double minx, const double miny, const double maxx, const double maxy,
         std::vector<uint32_t> levels) {
        if (!levels.size()) {
          levels.push_back(0);
          levels.push_back(1);
          levels.push_back(2);
        }

        check_coord(minx, miny, maxx, maxy);

        const vm::AABB2<vm::PointLL> bbox{minx, miny, maxx, maxy};
        std::vector<vb::GraphId> tile_ids;
        for (const auto level : levels) {
          check_level(level);
          const auto level_tile_ids = vb::TileHierarchy::levels().at(level).tiles.TileList(bbox);
          tile_ids.reserve(tile_ids.size() + level_tile_ids.size());
          for (const auto tid : level_tile_ids) {
            tile_ids.emplace_back(vb::GraphId{static_cast<uint32_t>(tid), level, 0});
          }
        }

        return tile_ids;
      },
      py::arg("minx"), py::arg("miny"), py::arg("maxx"), py::arg("maxy"),
      py::arg("levels") = std::vector<uint32_t>{});
}
} // namespace pyvalhalla
