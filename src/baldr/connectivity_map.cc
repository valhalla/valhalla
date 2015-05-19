#include "baldr/connectivity_map.h"
#include "baldr/json.h"
#include "baldr/graphtile.h"

#include <valhalla/midgard/pointll.h>
#include <boost/filesystem.hpp>
#include <list>

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

  json::MapPtr to_properties(const size_t color) {
    return json::map({
      {std::string("color"), static_cast<uint64_t>(color)}
    });
  }

  json::MapPtr to_geometry(const std::list<PointLL>& tiles) {
    auto multipoint = json::array({});
    for(const auto& tile : tiles)
      multipoint->emplace_back(json::array({json::fp_t{tile.first, 6}, json::fp_t{tile.second, 6}}));
    return json::map({
      {std::string("type"), std::string("MultiPoint")},
      {std::string("coordinates"), multipoint}
    });
  }

  json::MapPtr to_feature(const std::pair<size_t, std::list<PointLL> >& region) {
    return json::map({
      {std::string("type"), std::string("Feature")},
      {std::string("geometry"), to_geometry(region.second)},
      {std::string("properties"), to_properties(region.first)}
    });
  }

  std::string to_feature_collection(const std::unordered_map<size_t, std::list<PointLL> >& regions) {
    auto features = json::array({});
    for(const auto& region : regions)
      features->emplace_back(to_feature(region));
    std::stringstream ss;
    ss << *json::map({
      {std::string("type"), std::string("FeatureCollection")},
      {std::string("features"), features}
    });
    return ss.str();
  }
}

namespace valhalla {
  namespace baldr {
    connectivity_map_t::connectivity_map_t(const TileHierarchy& tile_hierarchy):tile_hierarchy(tile_hierarchy) {
      // Populate a map for each level of the tiles that exist
      for (const auto& tile_level : tile_hierarchy.levels()) {
        try {
          auto& level_colors = colors.insert({tile_level.first, std::unordered_map<uint32_t, size_t>{}}).first->second;
          boost::filesystem::path root_dir(tile_hierarchy.tile_dir() + '/' + std::to_string(tile_level.first) + '/');
          if(boost::filesystem::exists(root_dir) && boost::filesystem::is_directory(root_dir)) {
            for (boost::filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
              if (!boost::filesystem::is_directory(i->path())) {
                GraphId id = GraphTile::GetTileId(i->path().string(), tile_hierarchy);
                level_colors.insert({id.tileid(), 0});
              }
            }
          }
        }
        catch(...) {
        }
      }

      // All tiles have color 0 (not connected), go through each level and connect them
      for(auto& level_colors : colors) {
        auto level = tile_hierarchy.levels().find(level_colors.first);
        level->second.tiles.ColorMap(level_colors.second);
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
    std::string connectivity_map_t::to_geojson(const uint32_t hierarchy_level) const {
      //bail if we dont have the level
      auto bbox = tile_hierarchy.levels().find(hierarchy_level);
      auto level = colors.find(hierarchy_level);
      if(bbox == tile_hierarchy.levels().cend() || level == colors.cend())
        throw std::runtime_error("hierarchy level not found");

      //make a region map (inverse mapping of color to lists of tiles)
      //could cache this but shouldnt need to call it much
      std::unordered_map<size_t, std::list<PointLL> > regions;
      for(const auto& tile : level->second) {
        auto region = regions.find(tile.second);
        if(region == regions.end())
          regions.insert({tile.second, {bbox->second.tiles.Center(tile.first)}});
        else
          region->second.push_back(bbox->second.tiles.Center(tile.first));
      }
      //turn it into geojson
      return to_feature_collection(regions);
    }

    std::vector<size_t> connectivity_map_t::to_image(const uint32_t hierarchy_level) const {
      //bail if we dont have the level
      auto bbox = tile_hierarchy.levels().find(hierarchy_level);
      auto level = colors.find(hierarchy_level);
      if(bbox == tile_hierarchy.levels().cend() || level == colors.cend())
        throw std::runtime_error("hierarchy level not found");

      std::vector<size_t> tiles(bbox->second.tiles.nrows() * bbox->second.tiles.ncolumns(), static_cast<uint32_t>(0));
      for(size_t i = 0; i < tiles.size(); ++i) {
        const auto color = level->second.find(static_cast<uint32_t>(i));
        if(color != level->second.cend())
          tiles[i] = color->second;
      }

      return tiles;
    }
  }
}
