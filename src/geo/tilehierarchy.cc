#include "geo/tilehierarchy.h"
#include "geo/pointll.h"

namespace valhalla {
namespace geo {

TileHierarchy::TileLevel::TileLevel(const unsigned char level, const std::string& name, const Tiles& tiles)
  :level(level), name(name), tiles(tiles) {
}

bool TileHierarchy::TileLevel::operator<(const TileLevel& other) const {
  //if you name it the same and have the same level number they will
  //be considered the same when it comes to sorting
  if(level == other.level) {
    return name < other.name;
  }
  return level < other.level;
}

TileHierarchy::TileHierarchy(const boost::property_tree::ptree& pt) {
  //grab out other config information
  tile_dir_ = pt.get<std::string>("output.tile_dir");

  //grab out each tile level
  for(const auto& level : pt.get_child("output.levels")) {
    Tiles tiles(AABB2(PointLL(-90, -180), PointLL(90, 180)), level.second.get<float>("size"));
    levels_.emplace(TileLevel(
      level.second.get<unsigned char>("level"), level.second.get<std::string>("name"), tiles));
  }

  //if we didn't have any levels that is just not usable
  if(levels_.empty())
    throw std::runtime_error("Expected 1 or more levels in the tile hierarchy");
}

TileHierarchy::TileHierarchy(){}

const std::set<TileHierarchy::TileLevel>& TileHierarchy::levels() const {
  return levels_;
}

const std::string& TileHierarchy::tile_dir() const{
  return tile_dir_;
}

}
}
