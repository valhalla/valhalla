#include "baldr/graphid.h"
#include "baldr/tilehierarchy.h"
#include "midgard/aabb2.h"
#include "midgard/boost_geom_types.h"
#include "midgard/pointll.h"
#include "midgard/util.h"
#include "module.h"
#include "tile_id_utils.h"

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

using valhalla::bindings::check_coord;
using valhalla::bindings::check_level;

namespace pyvalhalla::baldr::utils {

void init_graphtile(nb::module_& m) {
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
      nb::arg("minx"), nb::arg("miny"), nb::arg("maxx"), nb::arg("maxy"),
      nb::arg("levels") = std::vector<uint32_t>{}, nb::call_guard<nb::gil_scoped_release>(),
      "Returns all tiles GraphIds for the specified levels (default: all),\n"
      "which intersect the bbox.\n\n"
      ":param minx: The bbox's minimum longitude.\n"
      ":param miny: The bbox's minimum latitude.\n"
      ":param maxx: The bbox's maximum longitude.\n"
      ":param maxy: The bbox's maximum latitude.\n"
      ":param levels: The hierarchy levels for which to find tiles.\n"
      ":returns: The list of tile GraphIds which intersect the bounding box\n"
      ":raises ValueError: When the level(s) or coord are invalid.");

  m.def(
      "get_tile_ids_from_ring",
      [](const std::vector<nb::tuple>& ring_coords,
         std::vector<uint32_t> levels) -> std::vector<vb::GraphId> {
        // parse binding-specific coordinate tuples into PointLL
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

        return valhalla::bindings::get_tile_ids_from_ring(std::move(ring), std::move(levels));
      },
      nb::arg("ring_coords"), nb::arg("levels") = std::vector<uint32_t>{},
      nb::call_guard<nb::gil_scoped_release>(),
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
