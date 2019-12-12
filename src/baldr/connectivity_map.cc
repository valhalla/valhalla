#include <iomanip>
#include <list>
#include <random>
#include <sstream>
#include <unordered_set>

#include "baldr/connectivity_map.h"
#include "baldr/graphreader.h"
#include "baldr/graphtile.h"
#include "baldr/json.h"
#include "baldr/tilehierarchy.h"
#include "midgard/constants.h"
#include "midgard/logging.h"
#include "midgard/pointll.h"
#include "midgard/util.h"

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {
/*
   { "type": "FeatureCollection",
    "features": [
      { "type": "Feature",
        "geometry": {"type": "Point", "coordinates": [102.0, 0.5]},
        "properties": {"prop0": "value0"}
        },
      { "type": "Feature",
        "geometry": {
          "type": "LineString",
          "coordinates": [
            [102.0, 0.0], [103.0, 1.0], [104.0, 0.0], [105.0, 1.0]
            ]
          },
        "properties": {
          "prop0": "value0",
          "prop1": 0.0
          }
        },
      { "type": "Feature",
         "geometry": {
           "type": "Polygon",
           "coordinates": [
             [ [100.0, 0.0], [101.0, 0.0], [101.0, 1.0],
               [100.0, 1.0], [100.0, 0.0] ]
             ]
         },
         "properties": {
           "prop0": "value0",
           "prop1": {"this": "that"}
           }
         }
       ]
     }
   */

json::MapPtr to_properties(uint64_t id, const std::string& color) {
  return json::map({
      {"fill", color},
      {"stroke", std::string("white")},
      {"stroke-width", static_cast<uint64_t>(1)},
      {"fill-opacity", json::fp_t{0.8, 1}},
      {"id", id},
  });
}

json::MapPtr to_geometry(const polygon_t& polygon) {
  auto coords = json::array({});
  // bool outer = true;
  for (const auto& ring : polygon) {
    auto ring_coords = json::array({});
    for (const auto& coord : ring) {
      // if (outer) {
      ring_coords->emplace_back(
          json::array({json::fp_t{coord.first, 6}, json::fp_t{coord.second, 6}}));
      /*} else {
        ring_coords->emplace_front(
            json::array({json::fp_t{coord.first, 6}, json::fp_t{coord.second, 6}}));
      }*/
    }
    coords->emplace_back(ring_coords);
    // outer = false;
  }
  return json::map({{"type", std::string("Polygon")}, {"coordinates", coords}});
}

json::MapPtr to_feature(const std::pair<size_t, polygon_t>& boundary, const std::string& color) {
  return json::map({{"type", std::string("Feature")},
                    {"geometry", to_geometry(boundary.second)},
                    {"properties", to_properties(boundary.first, color)}});
}

template <class T>
std::string to_feature_collection(const std::unordered_map<size_t, polygon_t>& boundaries,
                                  const std::multimap<size_t, size_t, T>& arities) {
  std::mt19937 generator(17);
  std::uniform_int_distribution<int> distribution(64, 192);
  auto features = json::array({});
  for (const auto& arity : arities) {
    std::stringstream hex;
    hex << "#" << std::hex << distribution(generator);
    hex << std::hex << distribution(generator);
    hex << std::hex << distribution(generator);
    features->emplace_back(to_feature(*boundaries.find(arity.second), hex.str()));
  }
  std::stringstream ss;
  ss << *json::map({{"type", std::string("FeatureCollection")}, {"features", features}});
  return ss.str();
}

} // namespace

namespace valhalla {
namespace baldr {
connectivity_map_t::connectivity_map_t(const boost::property_tree::ptree& pt) {
  // See what kind of tiles we are dealing with here by getting a graphreader
  GraphReader reader(pt);
  auto tiles = reader.GetTileSet();
  transit_level = TileHierarchy::levels().rbegin()->second.level + 1;

  // Quick hack to remove connectivity between known unconnected regions
  // The only land connection from north to south america is through
  // parque nacional de darien which has no passable ways, there are no ferries either
  // x = 409, y=391 = (391*1440 + 409)
  std::unordered_map<uint32_t, uint32_t> not_neighbors{{563449, 563450},
                                                       {563450, 563451},
                                                       {563451, 564891},
                                                       {564891, 564892},
                                                       {566331, 566332}};
  // TODO: actually check what neighbor tiles are reachable via looking at
  // edges end nodes instead of just doing a coloring based on proximity
  // then use this map as input to this singleton (via geojson?)

  // Populate a map for each level of the tiles that exist
  for (const auto& t : tiles) {
    auto& level_colors =
        colors.insert({t.level(), std::unordered_map<uint32_t, size_t>{}}).first->second;
    level_colors.insert({t.tileid(), 0});
  }

  // All tiles have color 0 (not connected), go through and connect
  // (build the ColorMap). Transit level uses local hierarchy tiles
  for (auto& color : colors) {
    if (color.first == transit_level) {
      TileHierarchy::levels().rbegin()->second.tiles.ColorMap(color.second, not_neighbors);
    } else {
      TileHierarchy::levels()
          .find(color.first)
          ->second.tiles.ColorMap(color.second,
                                  color.first == 2 ? not_neighbors : decltype(not_neighbors){});
    }
  }
}

size_t connectivity_map_t::get_color(const GraphId& id) const {
  auto level = colors.find(id.level());
  if (level == colors.cend()) {
    return 0;
  }
  auto color = level->second.find(id.tileid());
  if (color == level->second.cend()) {
    return 0;
  }
  return color->second;
}

std::unordered_set<size_t> connectivity_map_t::get_colors(uint32_t hierarchy_level,
                                                          const baldr::PathLocation& location,
                                                          float radius) const {

  std::unordered_set<size_t> result;
  auto level = colors.find(hierarchy_level);
  if (level == colors.cend()) {
    return result;
  }
  const auto& tiles = TileHierarchy::levels().find(hierarchy_level)->second.tiles;
  std::vector<const decltype(location.edges)*> edge_sets{&location.edges, &location.filtered_edges};
  for (const auto* edges : edge_sets) {
    for (const auto& edge : *edges) {
      // Get a list of tiles required within the radius of the projected point
      const auto& ll = edge.projected;
      DistanceApproximator approximator(ll);
      float latdeg = (radius / kMetersPerDegreeLat);
      float lngdeg = (radius / DistanceApproximator::MetersPerLngDegree(ll.lat()));
      AABB2<PointLL> bbox(Point2(ll.lng() - lngdeg, ll.lat() - latdeg),
                          Point2(ll.lng() + lngdeg, ll.lat() + latdeg));
      std::vector<int32_t> tilelist = tiles.TileList(bbox);
      for (auto& id : tilelist) {
        auto color = level->second.find(id);
        if (color != level->second.cend()) {
          result.emplace(color->second);
        }
      }
    }
  }
  return result;
}

std::string connectivity_map_t::to_geojson(const uint32_t hierarchy_level) const {
  // bail if we dont have the level
  uint32_t tile_level = (hierarchy_level == transit_level) ? transit_level - 1 : hierarchy_level;
  auto bbox = TileHierarchy::levels().find(tile_level);
  if (bbox == TileHierarchy::levels().cend()) {
    throw std::runtime_error("hierarchy level not found");
  }

  // make a region map (inverse mapping of color to lists of tiles)
  // could cache this but shouldnt need to call it much
  std::unordered_map<size_t, std::unordered_set<uint32_t>> regions;
  auto level = colors.find(hierarchy_level);
  if (level != colors.cend()) {
    for (const auto& tile : level->second) {
      auto region = regions.find(tile.second);
      if (region == regions.end()) {
        regions.emplace(tile.second, std::unordered_set<uint32_t>{tile.first});
      } else {
        region->second.emplace(tile.first);
      }
    }
  }

  // record the arity of each region so we can put the biggest ones first
  auto comp = [](const size_t& a, const size_t& b) { return a > b; };
  std::multimap<size_t, size_t, decltype(comp)> arities(comp);
  for (const auto& region : regions) {
    arities.emplace(region.second.size(), region.first);
  }

  // get the boundary of each region
  std::unordered_map<size_t, polygon_t> boundaries;
  for (const auto& arity : arities) {
    auto& region = *regions.find(arity.second);
    boundaries.emplace(arity.second, to_boundary(region.second, bbox->second.tiles));
  }

  // turn it into geojson
  return to_feature_collection<decltype(comp)>(boundaries, arities);
}

std::vector<size_t> connectivity_map_t::to_image(const uint32_t hierarchy_level) const {
  uint32_t tile_level = (hierarchy_level == transit_level) ? transit_level - 1 : hierarchy_level;
  auto bbox = TileHierarchy::levels().find(tile_level);
  if (bbox == TileHierarchy::levels().cend()) {
    throw std::runtime_error("hierarchy level not found");
  }

  std::vector<size_t> tiles(bbox->second.tiles.nrows() * bbox->second.tiles.ncolumns(),
                            static_cast<uint32_t>(0));
  auto level = colors.find(hierarchy_level);
  if (level != colors.cend()) {
    for (size_t i = 0; i < tiles.size(); ++i) {
      const auto color = level->second.find(static_cast<uint32_t>(i));
      if (color != level->second.cend()) {
        tiles[i] = color->second;
      }
    }
  }

  return tiles;
}
} // namespace baldr
} // namespace valhalla
