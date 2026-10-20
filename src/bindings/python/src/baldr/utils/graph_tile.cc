#include "baldr/graphid.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/boost_geom_types.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "module.h"
#include "tile_id_utils.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

using valhalla::bindings::check_coord;
using valhalla::bindings::check_level;

namespace pyvalhalla::baldr::utils {

void init_graphtile(nb::module_& m) {
  nb::class_<vm::PointLL>(
      m, "PointLL",
      "Longitude, latitude point in degrees (lon first, lat second).")
      .def(
          "__init__",
          [](vm::PointLL* self, double lon, double lat) { new (self) vm::PointLL{lon, lat}; },
          nb::arg("lon"), nb::arg("lat"),
          "Constructs a PointLL from longitude and latitude in degrees.")
      .def("lng", &vm::PointLL::lng, "Gets the longitude in degrees.")
      .def("lat", &vm::PointLL::lat, "Gets the latitude in degrees.")
      .def("is_valid", &vm::PointLL::IsValid,
           "Returns false if the coordinates hold the invalid sentinel.")
      .def("__repr__", [](const vm::PointLL& p) {
        return nb::str(("<PointLL(" + std::to_string(p.lng()) + ", " + std::to_string(p.lat()) + ")>").c_str());
      });

  nb::class_<vm::AABB2<vm::PointLL>>(
      m, "AABB2",
      "Axis-aligned bounding box of PointLL coordinates (lon/lat degrees).")
      .def(
          "__init__",
          [](vm::AABB2<vm::PointLL>* self, double minx, double miny, double maxx, double maxy) {
            new (self) vm::AABB2<vm::PointLL>{minx, miny, maxx, maxy};
          },
          nb::arg("minx"), nb::arg("miny"), nb::arg("maxx"), nb::arg("maxy"),
          "Constructs a bounding box from min/max lon/lat values.")
      .def(
          "__init__",
          [](vm::AABB2<vm::PointLL>* self, const vm::PointLL& minpt,
             const vm::PointLL& maxpt) { new (self) vm::AABB2<vm::PointLL>{minpt, maxpt}; },
          nb::arg("minpt"), nb::arg("maxpt"),
          "Constructs a bounding box from minimum and maximum points.")
      .def("minx", &vm::AABB2<vm::PointLL>::minx, "Gets the minimum x (lon).")
      .def("maxx", &vm::AABB2<vm::PointLL>::maxx, "Gets the maximum x (lon).")
      .def("miny", &vm::AABB2<vm::PointLL>::miny, "Gets the minimum y (lat).")
      .def("maxy", &vm::AABB2<vm::PointLL>::maxy, "Gets the maximum y (lat).")
      .def("minpt", &vm::AABB2<vm::PointLL>::minpt, "Gets the minimum point.")
      .def("maxpt", &vm::AABB2<vm::PointLL>::maxpt, "Gets the maximum point.")
      .def("__repr__", [](const vm::AABB2<vm::PointLL>& b) {
        return nb::str(("<AABB2(" + std::to_string(b.minx()) + ", " + std::to_string(b.miny()) +
                         ", " + std::to_string(b.maxx()) + ", " + std::to_string(b.maxy()) + ")>").c_str());
      });

  nb::class_<vb::TileHierarchy>(
      m, "TileHierarchy",
      "Static hierarchy of routing tiles. Query one level at a time with "
      "GetGraphIds(bbox, level).")
      .def_static(
          "GetGraphId",
          [](const vm::PointLL& point, const uint32_t level) {
            check_level(level);
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
            check_level(level);
            return vb::TileHierarchy::GetGraphIds(bbox, static_cast<uint8_t>(level));
          },
          nb::arg("bbox"), nb::arg("level"), nb::call_guard<nb::gil_scoped_release>(),
          "Get all tile GraphIds intersecting the bbox at a single hierarchy level.\n\n"
          ":param bbox: The AABB2 bounding box of tiles to find.\n"
          ":param level: The hierarchy level.\n"
          ":returns: The list of tile GraphIds which intersect the bounding box.\n"
          ":raises ValueError: When the level is invalid.");

  m.def(
      "get_tile_base_lon_lat",
      [](const vb::GraphId& graph_id) {
        const auto& tiles_at_level = vb::TileHierarchy::levels().at(graph_id.level());
        const auto pt = tiles_at_level.tiles.Base(graph_id.tileid());

        return nb::make_tuple(pt.x(), pt.y());
      },
      nb::arg("graph_id"),
      "Get the geographic coordinate of the south-western corner of this graph_id's tile.\n\n"
      ":param graph_id: The tile's or object's GraphId\n"
      ":returns: The lon/lat coordinate of the SW corner of the tile");

  m.def(
      "get_tile_id_from_lon_lat",
      [](const uint32_t level, const nb::tuple& coord) {
        if (nb::len(coord) != 2) {
          throw nb::value_error("Invalid coordinate size, must be 2");
        }

        check_level(level);

        auto x = nb::cast<double>(coord[0]);
        auto y = nb::cast<double>(coord[1]);
        check_coord(x, y, x, y);

        return vb::TileHierarchy::GetGraphId(vm::PointLL{x, y}, static_cast<uint8_t>(level));
      },
      nb::arg("level"), nb::arg("coord"),
      "Get the tile at this hierarchy level and geographic coordinate.\n\n"
      ":param level: The hierarchy level of the searched tiles.\n"
      ":param coord: The geographic coordinate to intersect with tiles.\n"
      ":returns: GraphId of found tile.\n"
      ":raises ValueError: When the level or coord are invalid.");

  m.def(
      "get_tile_ids_from_ring",
      [](const std::vector<nb::tuple>& ring_coords,
         std::vector<uint32_t> levels) -> std::vector<vb::GraphId> {
        // Parse binding-specific coordinate tuples into PointLL.
        std::vector<vm::PointLL> ring;
        ring.reserve(ring_coords.size());

        for (const auto& coord : ring_coords) {
          if (nb::len(coord) != 2) {
            throw nb::value_error("Each coordinate must have 2 elements (lon, lat)");
          }

          auto x = nb::cast<double>(coord[0]);
          auto y = nb::cast<double>(coord[1]);
          check_coord(x, y, x, y);
          ring.push_back({x, y});
        }

        nb::gil_scoped_release release;
        return valhalla::bindings::get_tile_ids_from_ring(std::move(ring), std::move(levels));
      },
      nb::arg("ring_coords"), nb::arg("levels") = std::vector<uint32_t>{},
      "Returns all tile GraphIds for the specified levels (default: all),\n"
      "which intersect or are contained within the polygon ring. The ring is\n"
      "assumed and coerced to be an outer ring. It's automatically closed if\n"
      "the last coordinate does not match the first.\n\n"
      ":param ring_coords: List of (lon, lat) tuples forming a closed ring (polygon boundary).\n"
      "               Must have at least 3 coordinates.\n"
      ":param levels: The hierarchy levels for which to find tiles.\n"
      ":returns: The list of tile GraphIds which intersect or are inside the ring.\n"
      ":raises ValueError: When the level(s), coords, or ring size are invalid.");
}

} // namespace pyvalhalla::baldr::utils
