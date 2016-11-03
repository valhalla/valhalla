#include "baldr/connectivity_map.h"
#include "baldr/json.h"
#include "baldr/graphtile.h"
#include "baldr/graphreader.h"

#include <valhalla/midgard/pointll.h>
#include <boost/filesystem.hpp>
#include <list>
#include <iomanip>
#include <random>
#include <sstream>
#include <unordered_set>

#include <valhalla/midgard/logging.h>

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

  using ring_t = std::list<PointLL>;
  using polygon_t = std::list<ring_t>;
  json::MapPtr to_geometry(const polygon_t& polygon) {
    auto coords = json::array({});
    bool outer = true;
    for(const auto& ring : polygon) {
      auto ring_coords = json::array({});
      for(const auto& coord : ring) {
        if(outer)
          ring_coords->emplace_back(json::array({json::fp_t{coord.first, 6}, json::fp_t{coord.second, 6}}));
        else
          ring_coords->emplace_front(json::array({json::fp_t{coord.first, 6}, json::fp_t{coord.second, 6}}));
      }
      coords->emplace_back(ring_coords);
      outer = false;
    }
    return json::map({
      {"type", std::string("Polygon")},
      {"coordinates", coords}
    });
  }

  json::MapPtr to_feature(const std::pair<size_t, polygon_t>& boundary, const std::string& color) {
    return json::map({
      {"type", std::string("Feature")},
      {"geometry", to_geometry(boundary.second)},
      {"properties", to_properties(boundary.first, color)}
    });
  }

  template <class T>
  std::string to_feature_collection(const std::unordered_map<size_t, polygon_t>& boundaries, const std::multimap<size_t, size_t, T>& arities) {
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(64,192);
    auto features = json::array({});
    for(const auto& arity : arities) {
      std::stringstream hex;
      hex << "#" << std::hex << distribution(generator);
      hex << std::hex << distribution(generator);
      hex << std::hex << distribution(generator);
      features->emplace_back(to_feature(*boundaries.find(arity.second), hex.str()));
    }
    std::stringstream ss;
    ss << *json::map({
      {"type", std::string("FeatureCollection")},
      {"features", features}
    });
    return ss.str();
  }

  polygon_t to_boundary(const std::pair<size_t, std::unordered_set<uint32_t> >& region, const Tiles<PointLL>& tiles) {
    //do we have this tile in this region
    auto member = [&region](int32_t tile) {
      return region.second.find(tile) != region.second.cend();
    };
    //get the neighbor tile giving -1 if no neighbor
    auto neighbor = [&tiles](int32_t tile, int side) -> int32_t {
      if(tile == -1) return -1;
      auto rc = tiles.GetRowColumn(tile);
      switch(side) {
        case 0: return rc.second == 0 ? -1 : tile - 1;
        case 1: return rc.first == 0 ? -1 : tile - tiles.ncolumns();
        case 2: return rc.second == tiles.ncolumns() - 1 ? -1 : tile + 1;
        case 3: return rc.first == tiles.nrows() - 1 ? -1 : tile + tiles.ncolumns();
      }
    };
    //get the beginning coord of the counter clockwise winding given edge of the given tile
    auto coord = [&tiles](uint32_t tile, int side) -> PointLL {
      auto box = tiles.TileBounds(tile);
      switch(side) {
        case 0: return PointLL(box.minx(), box.maxy());
        case 1: return box.minpt();
        case 2: return PointLL(box.maxx(), box.miny());
        case 3: return box.maxpt();
      }
    };
    //trace a ring of the polygon
    polygon_t polygon;
    std::array<std::unordered_set<uint32_t>, 4> used;
    auto trace = [&member, &neighbor, &coord, &polygon, &used](uint32_t start_tile, int start_side, bool ccw) {
      auto tile = start_tile;
      auto side = start_side;
      polygon.emplace_back();
      auto& ring = polygon.back();
      //walk until you see the starting edge again
      do {
        //add this edges geometry
        if(ccw)
          ring.push_back(coord(tile, side));
        else
          ring.push_front(coord(tile, side));
        auto inserted = used[side].insert(tile);
        if(!inserted.second)
          throw std::logic_error("Any tile edge can only be used once as part of the geometry");
        //we need to go to the first existing neighbor tile following our winding
        //starting with the one on the other side of the current side
        auto adjc = neighbor(tile, (side + 1) % 4);
        auto diag = neighbor(adjc, side);
        if(member(diag)){
          tile = diag;
          side = (side + 3) % 4;
        }//next one keep following winding
        else if(member(adjc)){
          tile = adjc;
        }//if neither of those were there we stay on this tile and go to the next side
        else {
          side = (side + 1) % 4;
        }
      } while(tile != start_tile || side != start_side);
    };

    //the smallest numbered tile has a left edge on the outer ring of the polygon
    auto start_tile = *region.second.cbegin();
    int start_side = 0;
    for(auto tile : region.second)
      if(tile < start_tile)
        start_tile = tile;

    //trace the outer
    trace(start_tile, start_side, true);

    //trace the inners
    for(auto start_tile : region.second) {
      //if the neighbor isnt a member and we didnt already use the side between them
      for(start_side = 0; start_side < 4; ++start_side) {
        if(!member(neighbor(start_tile, start_side)) &&
            used[start_side].find(start_tile) == used[start_side].cend()) {
          //build the inner ring
          if(start_side != -1)
            trace(start_tile, start_side, false);
        }
      }
    }

    //close all the rings
    for(auto& ring : polygon)
      ring.push_back(ring.front());

    //give it back
    return polygon;
  }
}

namespace valhalla {
  namespace baldr {
    connectivity_map_t::connectivity_map_t(const boost::property_tree::ptree& pt)
      :tile_hierarchy(pt.get<std::string>("tile_dir")) {
      // See what kind of tiles we are dealing with here by getting a graphreader
      GraphReader reader(pt);
      auto tiles = reader.GetTileSet();
      transit_level = tile_hierarchy.levels().rbegin()->second.level + 1;

      // Populate a map for each level of the tiles that exist
      for(const auto& t : tiles) {
        auto& level_colors = colors.insert({t.level(), std::unordered_map<uint32_t, size_t>{}}).first->second;
        level_colors.insert({t.tileid(), 0});
      }

      // All tiles have color 0 (not connected), go through and connect
      // (build the ColorMap). Transit level uses local hierarchy tiles
      for (auto& color : colors) {
        if (color.first == transit_level)
          tile_hierarchy.levels().rbegin()->second.tiles.ColorMap(color.second);
        else
          tile_hierarchy.levels().find(color.first)->second.tiles.ColorMap(color.second);
      }
    }

    size_t connectivity_map_t::get_color(const GraphId& id) const {
      auto level = colors.find(id.level());
      if(level == colors.cend())
        return 0;
      auto color = level->second.find(id.tileid());
      if(color == level->second.cend())
        return 0;
      return color->second;
    }

    std::unordered_set<size_t> connectivity_map_t::get_colors(uint32_t hierarchy_level,
      const baldr::PathLocation& location, float radius) const {

      std::unordered_set<size_t> result;
      auto level = colors.find(hierarchy_level);
      if(level == colors.cend())
        return result;
      const auto& tiles = tile_hierarchy.levels().find(hierarchy_level)->second.tiles;
      for(const auto& edge : location.edges) {
        //TODO: generate more ids by intersecting this circle with the tiles object
        //take the edge.projected and subtract radius in the x and y dimensions
        //convert that into a tileid and get its col,row tile coords that will start the for loop
        //add radius in the x and y to edge.projected to get the ending tile col,row for the for loop
        //then while iterating create a tile for each tile col,row pair, get its aabb2 and call
        //aabb2::intersects(edge.projected, radius), if it returns true, get the color as below
        auto id = tiles.TileId(edge.projected);
        auto color = level->second.find(id);
        if(color != level->second.cend())
          result.emplace(color->second);
      }
      return result;
    }

    std::string connectivity_map_t::to_geojson(const uint32_t hierarchy_level) const {
      //bail if we dont have the level
      auto bbox = tile_hierarchy.levels().find(
        hierarchy_level == transit_level ? transit_level - 1 : hierarchy_level);
      if(bbox == tile_hierarchy.levels().cend())
        throw std::runtime_error("hierarchy level not found");

      //make a region map (inverse mapping of color to lists of tiles)
      //could cache this but shouldnt need to call it much
      std::unordered_map<size_t, std::unordered_set<uint32_t> > regions;
      auto level = colors.find(hierarchy_level);
      if(level != colors.cend()) {
        for(const auto& tile : level->second) {
          auto region = regions.find(tile.second);
          if(region == regions.end())
            regions.emplace(tile.second, std::unordered_set<uint32_t>{tile.first});
          else
            region->second.emplace(tile.first);
        }
      }

      //record the arity of each region so we can put the biggest ones first
      auto comp = [](const size_t& a, const size_t& b){return a > b;};
      std::multimap<size_t, size_t, decltype(comp)> arities(comp);
      for(const auto& region : regions)
        arities.emplace(region.second.size(), region.first);

      //get the boundary of each region
      std::unordered_map<size_t, polygon_t> boundaries;
      for(const auto& arity : arities) {
        auto& region = *regions.find(arity.second);
        boundaries.emplace(arity.second, to_boundary(region, bbox->second.tiles));
      }

      //turn it into geojson
      return to_feature_collection<decltype(comp)>(boundaries, arities);
    }

    std::vector<size_t> connectivity_map_t::to_image(const uint32_t hierarchy_level) const {
      uint32_t tile_level = (hierarchy_level == transit_level) ? transit_level - 1 : hierarchy_level;
      auto bbox = tile_hierarchy.levels().find(tile_level);
      if (bbox == tile_hierarchy.levels().cend())
        throw std::runtime_error("hierarchy level not found");

      std::vector<size_t> tiles(bbox->second.tiles.nrows() * bbox->second.tiles.ncolumns(), static_cast<uint32_t>(0));
      auto level = colors.find(hierarchy_level);
      if (level != colors.cend()) {
        for(size_t i = 0; i < tiles.size(); ++i) {
          const auto color = level->second.find(static_cast<uint32_t>(i));
          if(color != level->second.cend())
            tiles[i] = color->second;
        }
      }

      return tiles;
    }
  }
}
