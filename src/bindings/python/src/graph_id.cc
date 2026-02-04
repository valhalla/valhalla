#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"
#include "graph_utils_module.h"
#include "midgard/aabb2.h"
#include "midgard/boost_geom_types.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

#include <boost/geometry/algorithms/within.hpp>
#include <boost/property_tree/ptree.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <fstream>
#include <queue>
#include <sstream>

namespace nb = nanobind;
namespace vb = valhalla::baldr;
namespace vm = valhalla::midgard;

namespace pyvalhalla {

// road levels + transit
const uint32_t MAX_LEVEL = vb::TileHierarchy::levels().back().level + 1;

auto check_level = [](const uint32_t level) {
  if (level >= MAX_LEVEL) {
    std::string msg = "We only support " + std::to_string(MAX_LEVEL) + " hierarchy levels.";
    throw nb::value_error(msg.data());
  }
};

auto check_coord = [](const double minx, const double miny, const double maxx, const double maxy) {
  if (minx < -180. || maxx > 180. || miny < -90. || maxy > 90.) {
    throw nb::value_error("Invalid coordinate, remember it's (lon, lat)");
  }
};

void init_graphid(nb::module_& m) {
  nb::class_<vb::GraphId>(m, "GraphId")
      .def(nb::init())
      .def(nb::init<uint32_t, uint32_t, uint32_t>(), nb::arg("tileid"), nb::arg("level"),
           nb::arg("id"))
      .def(nb::init<uint64_t>())
      .def(nb::init<std::string>())
      .def_ro("value", &vb::GraphId::value)
      .def("tileid", &vb::GraphId::tileid)
      .def("level", &vb::GraphId::level)
      .def("id", &vb::GraphId::id)
      .def("is_valid", &vb::GraphId::is_valid)
      .def("tile_base", &vb::GraphId::tile_base)
      .def("tile_value", &vb::GraphId::tile_value)
      .def(nb::self + uint64_t())              // operator+(uint64_t)
      .def(nb::self += uint32_t())             // operator+=(uint32_t)
      .def(nb::self == nb::self)               // operator==(const GraphId&)
      .def(nb::self != nb::self)               // operator!=(const GraphId&)
      .def("__bool__", &vb::GraphId::is_valid) // operator bool
      .def("__repr__",
           [](const vb::GraphId& graph_id) { return "<GraphId(" + std::to_string(graph_id) + ")>"; })
      // pickling support auto-provides copy/deepcopy support
      .def("__getstate__", [](const vb::GraphId& gid) { return std::make_tuple(gid.value); })
      .def("__setstate__", [](vb::GraphId& self, const std::tuple<uint64_t>& state) {
        new (&self) vb::GraphId(std::get<0>(state));
      });

  m.def(
      "get_tile_base_lon_lat",
      [](const vb::GraphId& graph_id) {
        const auto& tiles_at_level = vb::TileHierarchy::levels().at(graph_id.level());
        const auto pt = tiles_at_level.tiles.Base(graph_id.tileid());

        return nb::make_tuple(pt.x(), pt.y());
      },
      nb::arg("graph_id"));

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
      nb::arg("level"), nb::arg("coord"));

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
      nb::arg("levels") = std::vector<uint32_t>{});

  m.def(
      "get_tile_ids_from_ring",
      [](const std::vector<nb::tuple>& ring_coords, std::vector<uint32_t> levels) {
        if (ring_coords.size() < 3) {
          throw nb::value_error("Ring must have at least 3 coordinates");
        }

        if (!levels.size()) {
          levels.push_back(0);
          levels.push_back(1);
          levels.push_back(2);
        }

        // build a boost geometry ring from the coordinate tuples
        vm::bg::ring_ll_t ring;
        ring.reserve(ring_coords.size() + 1);
        for (const auto& coord : ring_coords) {
          if (nb::len(coord) != 2) {
            throw nb::value_error("Each coordinate must have 2 elements (lon, lat)");
          }
          auto x = nb::cast<double>(coord[0]);
          auto y = nb::cast<double>(coord[1]);
          check_coord(x, y, x, y);
          ring.push_back({x, y});
        }

        // close open rings
        if (ring.front().lng() != ring.back().lng() || ring.front().lat() != ring.back().lat()) {
          ring.push_back(ring.front());
        }
        // reverse if counter-clockwise
        if (vm::polygon_area(ring) > 0) {
          std::reverse(ring.begin(), ring.end());
        }

        std::vector<vb::GraphId> result;
        for (const auto level : levels) {
          check_level(level);
          const auto& tiles = vb::TileHierarchy::get_tiling(static_cast<uint8_t>(level));

          // find tiles whose bins intersect the ring boundary
          auto intersected = tiles.Intersect(ring);
          std::unordered_set<int32_t> boundary_tiles;
          boundary_tiles.reserve(intersected.size());
          for (const auto& [tile_id, bins] : intersected) {
            boundary_tiles.insert(tile_id);
          }

          // find a starting tile adjacent to a boundary tile that is fully inside
          std::optional<int32_t> start_tile;
          for (const auto bt : boundary_tiles) {
            if (start_tile)
              break;
            for (int32_t neighbor : {tiles.RightNeighbor(bt), tiles.LeftNeighbor(bt),
                                     tiles.TopNeighbor(bt), tiles.BottomNeighbor(bt)}) {
              // if bt is on the edge of the tiling, skip
              if (neighbor == bt)
                continue;
              // if neighbor is a boundary tile, skip
              else if (boundary_tiles.count(neighbor))
                continue;
              // if neighbor is inside the ring, we found our start tile
              else if (boost::geometry::within(tiles.Center(neighbor), ring)) {
                start_tile = neighbor;
                break;
              }
            }
          }

          // then flood fill to find tiles completely inside the ring
          std::unordered_set<int32_t> contained_tiles;
          if (start_tile) {
            std::queue<int32_t> queue;
            queue.push(*start_tile);
            contained_tiles.insert(*start_tile);
            size_t n = 0;

            // don't bail on planetwide polygon (>1.1 Mio tiles)
            while (!queue.empty() && n++ < 1.2e6) {
              auto current = queue.front();
              queue.pop();
              for (int32_t neighbor : {tiles.RightNeighbor(current), tiles.LeftNeighbor(current),
                                       tiles.TopNeighbor(current), tiles.BottomNeighbor(current)}) {
                // if bt is on the edge of the tiling, skip
                if (neighbor == current)
                  continue;
                // if neighbor is a boundary tile, skip
                else if (boundary_tiles.count(neighbor))
                  continue;
                // if neighbor is already contained, skip
                else if (contained_tiles.count(neighbor))
                  continue;
                contained_tiles.insert(neighbor);
                queue.push(neighbor);
              }
            }
          }

          result.reserve(result.size() + boundary_tiles.size() + contained_tiles.size());
          for (const auto tile_id : boundary_tiles) {
            result.emplace_back(static_cast<uint32_t>(tile_id), level, 0);
          }
          for (const auto tile_id : contained_tiles) {
            result.emplace_back(static_cast<uint32_t>(tile_id), level, 0);
          }
        }

        return result;
      },
      nb::arg("ring_coords"), nb::arg("levels") = std::vector<uint32_t>{});

  // GraphUtils class - manages GraphReader for efficient edge access
  nb::class_<vb::GraphReader>(m, "_GraphUtils")
      .def(
          "__init__",
          [](vb::GraphReader* self, const std::string& config) {
            // Parse the config - handle both file paths and JSON strings
            boost::property_tree::ptree pt;
            try {
              std::ifstream file(config);
              if (file.good()) {
                rapidjson::read_json(file, pt);
              } else {
                std::istringstream stream(config);
                rapidjson::read_json(stream, pt);
              }
            } catch (...) { throw std::runtime_error("Failed to parse config JSON"); }

            // Configure logging like Actor does (suppress WARN/INFO messages)
            auto logging_subtree = pt.get_child_optional("mjolnir.logging");
            if (logging_subtree) {
              auto logging_config =
                  vm::ToMap<const boost::property_tree::ptree&,
                            std::unordered_map<std::string, std::string>>(logging_subtree.get());
              vm::logging::Configure(logging_config);
            }

            // Create GraphReader with mjolnir subtree
            new (self) vb::GraphReader(pt.get_child("mjolnir"));
          },
          nb::arg("config"))
      .def(
          "get_edge_shape",
          [](vb::GraphReader& self, const vb::GraphId& edge_id) {
            // Get the tile containing this edge
            auto tile = self.GetGraphTile(edge_id);
            if (!tile) {
              throw std::runtime_error("Tile not found for edge " + std::to_string(edge_id));
            }

            // Get the directed edge
            const auto* directed_edge = tile->directededge(edge_id);
            if (!directed_edge) {
              throw std::runtime_error("Edge not found: " + std::to_string(edge_id));
            }

            // Get the edge info and shape
            auto edge_info = tile->edgeinfo(directed_edge);
            auto shape = edge_info.shape();

            // Reverse the shape if the edge is not forward
            if (!directed_edge->forward()) {
              std::reverse(shape.begin(), shape.end());
            }

            // Convert to list of (lon, lat) tuples
            std::vector<nb::tuple> result;
            result.reserve(shape.size());
            for (const auto& pt : shape) {
              result.push_back(nb::make_tuple(pt.lng(), pt.lat()));
            }

            return result;
          },
          nb::arg("edge_id"));
}
} // namespace pyvalhalla
