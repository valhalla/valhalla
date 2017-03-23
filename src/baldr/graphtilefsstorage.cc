#include "baldr/graphtilefsstorage.h"
#include "baldr/graphtile.h"
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/sequence.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/logging.h>

#include <cmath>
#include <locale>
#include <iomanip>
#include <mutex>
#include <sys/stat.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/copy.hpp>

namespace {
  struct dir_facet : public std::numpunct<char> {
   protected:
    virtual char do_thousands_sep() const {
        return '/';
    }

    virtual std::string do_grouping() const {
        return "\03";
    }
  };
  template <class numeric_t>
  size_t digits(numeric_t number) {
    size_t digits = (number < 0 ? 1 : 0);
    while (static_cast<long long int>(number)) {
        number /= 10;
        digits++;
    }
    return digits;
  }
  const std::locale dir_locale(std::locale("C"), new dir_facet());
  const valhalla::midgard::AABB2<valhalla::midgard::PointLL> world_box(valhalla::midgard::PointLL(-180, -90), valhalla::midgard::PointLL(180, 90));
}

namespace valhalla {
namespace baldr {

struct GraphTileFsStorage::tile_extract_t : public midgard::tar {
  tile_extract_t(const boost::property_tree::ptree& pt):tar(pt.get<std::string>("tile_extract","")) {
    //if you really meant to load it
    if(pt.get_optional<std::string>("tile_extract")) {
      //map files to graph ids
      for(auto& c : contents) {
        try {
          auto id = GraphTile::GetTileId(c.first);
          tiles[id] = std::make_pair(const_cast<char*>(c.second.first), c.second.second);
        }
        catch(...){}
      }
      //couldn't load it
      if(tiles.empty()) {
        LOG_WARN("Tile extract could not be loaded");
      }//loaded ok but with possibly bad blocks
      else {
        LOG_INFO("Tile extract successfully loaded");
        if(corrupt_blocks)
          LOG_WARN("Tile extract had " + std::to_string(corrupt_blocks) + " corrupt blocks");
      }
    }
  }
  // TODO: dont remove constness, and actually make graphtile read only?
  std::unordered_map<uint64_t, std::pair<char*, size_t> > tiles;
};

std::shared_ptr<const GraphTileFsStorage::tile_extract_t> GraphTileFsStorage::get_extract_instance(const boost::property_tree::ptree& pt) {
  static std::mutex mutex;
  static std::vector<std::pair<boost::property_tree::ptree, std::shared_ptr<const GraphTileFsStorage::tile_extract_t>>> instances;

  std::lock_guard<std::mutex> lock(mutex);
  for (const auto& instance : instances) {
    if (instance.first == pt) {
      return instance.second;
    }
  }
  std::shared_ptr<const GraphTileFsStorage::tile_extract_t> tile_extract(new GraphTileFsStorage::tile_extract_t(pt));
  instances.emplace_back(pt, tile_extract);
  return tile_extract;
}

GraphTileFsStorage::GraphTileFsStorage(const boost::property_tree::ptree& pt)
    : tile_dir_(pt.get<std::string>("tile_dir")),
      tile_extract_(get_extract_instance(pt)) {
}

std::unordered_set<GraphId> GraphTileFsStorage::FindTiles(const TileHierarchy& tile_hierarchy) const {
  std::unordered_set<GraphId> graphids;
  if (tile_extract_->tiles.size()) {
    for (const auto& t : tile_extract_->tiles) {
      graphids.emplace(t.first);
    }
  } else {
    // Set the transit level
    auto transit_level = tile_hierarchy.levels().rbegin()->second.level + 1;

    // Populate a map for each level of the tiles that exist
    for (uint32_t tile_level = 0; tile_level <= transit_level; tile_level++) {
      boost::filesystem::path root_dir(tile_dir_ + '/' + std::to_string(tile_level) + '/');
      if(boost::filesystem::exists(root_dir) && boost::filesystem::is_directory(root_dir)) {
        for (boost::filesystem::recursive_directory_iterator i(root_dir), end; i != end; ++i) {
          if (!boost::filesystem::is_directory(i->path())) {
            try {
              //add it if it can be parsed as a valid tile file name
              graphids.emplace(GetTileId(i->path().string(), tile_dir_));
            } catch (...) { }
          }
        }
      }
    }
  }
  return graphids;
}

bool GraphTileFsStorage::DoesTileExist(const GraphId& graphid, const TileHierarchy& tile_hierarchy) const {
  if(tile_extract_->tiles.find(graphid) != tile_extract_->tiles.cend())
    return true;

  std::string file_location = tile_dir_ + "/" + FileSuffix(graphid.Tile_Base(), tile_hierarchy);
  struct stat buffer;
  return stat(file_location.c_str(), &buffer) == 0;
}

bool GraphTileFsStorage::ReadTile(const GraphId& graphid, const TileHierarchy& tile_hierarchy, std::vector<char>& tile_data) const {
  if (tile_extract_->tiles.size()) {
    auto it = tile_extract_->tiles.find(graphid);
    if (it != tile_extract_->tiles.cend()) {
      tile_data = std::vector<char>(it->second.first, it->second.first + it->second.second);
      return true;
    }
  } else {
    // Open to the end of the file so we can immediately get size;
    std::string file_location = tile_dir_ + "/" + FileSuffix(graphid.Tile_Base(), tile_hierarchy);
    std::ifstream file(file_location, std::ios::in | std::ios::binary | std::ios::ate);
    if (file.is_open()) {
      // Read binary file into memory. TODO - protect against failure to
      // allocate memory
      auto filesize = file.tellg();
      tile_data.resize(filesize);
      file.seekg(0, std::ios::beg);
      file.read(tile_data.data(), filesize);
      file.close();
      return !file.fail();
    }
    else {
      std::ifstream file(file_location + ".gz", std::ios::in | std::ios::binary | std::ios::ate);
      if (file.is_open()) {
        // Pre-allocate assuming 3.25:1 compression ratio (based on scanning some large NA tiles)
        size_t filesize = file.tellg();
        file.seekg(0, std::ios::beg);
        tile_data.reserve(filesize * 3 + filesize/4);  // TODO: read the gzip footer and get the real size?

        // Decompress tile into memory
        boost::iostreams::filtering_ostream os;
        os.push(boost::iostreams::gzip_decompressor());
        os.push(boost::iostreams::back_inserter(tile_data));
        boost::iostreams::copy(file, os);
        return !file.fail();
      }
    }
  }
  return false;
}

bool GraphTileFsStorage::ReadTileRealTimeSpeeds(const GraphId& graphid, const TileHierarchy& tile_hierarchy, std::vector<uint8_t>& rts_data) const {
  // Try to load the speeds file
  auto tileid = graphid.tileid();
  std::string traffic_dir = tile_dir_ + "/traffic/";
  std::string file_location = traffic_dir + std::to_string(tileid) + ".spd";
  std::ifstream rtsfile(file_location, std::ios::binary | std::ios::in | std::ios::ate);
  if (rtsfile.is_open()) {
    auto filesize = rtsfile.tellg();
    LOG_INFO("Load real time speeds: count = " + std::to_string(filesize));
    rts_data.resize(filesize);
    rtsfile.seekg(0, std::ios::beg);
    rtsfile.read((char*)(&rts_data.front()), filesize);
    rtsfile.close();
    return !rtsfile.fail();
  }
  return false;
}

const std::string& GraphTileFsStorage::GetTileDir() const {
  return tile_dir_;
}

// Get the tile Id given the full path to the file.
GraphId GraphTileFsStorage::GetTileId(const std::string& fname, const std::string& tile_dir) {
  //strip off the unuseful part
  auto pos = fname.find(tile_dir);
  if(pos == std::string::npos)
    throw std::runtime_error("File name for tile does not match hierarchy root dir");
  auto name = fname.substr(pos + tile_dir.size());
  boost::algorithm::trim_if(name, boost::is_any_of("/.gph"));

  //split on slash
  std::vector<std::string> tokens;
  boost::split(tokens, name, boost::is_any_of("/"));

  //need at least level and id
  if(tokens.size() < 2)
    throw std::runtime_error("Invalid tile path");

  // Compute the Id
  uint32_t id = 0;
  uint32_t multiplier = std::pow(1000, tokens.size() - 2);
  bool first = true;
  for(const auto& token : tokens) {
    if(first) {
      first = false;
      continue;
    }
    id += std::atoi(token.c_str()) * multiplier;
    multiplier /= 1000;
  }
  uint32_t level = std::atoi(tokens.front().c_str());
  return {id, level, 0};
}

std::string GraphTileFsStorage::FileSuffix(const GraphId& graphid, const TileHierarchy& tile_hierarchy) {
  /*
  if you have a graphid where level == 8 and tileid == 24134109851
  you should get: 8/024/134/109/851.gph
  since the number of levels is likely to be very small this limits
  the total number of objects in any one directory to 1000, which is an
  empirically derived good choice for mechanical harddrives
  this should be fine for s3 (even though it breaks the rule of most
  unique part of filename first) because there will be just so few
  objects in general in practice
  */

  //figure the largest id for this level
  auto level = tile_hierarchy.levels().find(graphid.level());
  if(level == tile_hierarchy.levels().end() &&
     graphid.level() == ((tile_hierarchy.levels().rbegin())->second.level + 1))
    level = tile_hierarchy.levels().begin();

  if(level == tile_hierarchy.levels().end())
    throw std::runtime_error("Could not compute FileSuffix for non-existent level");

  const uint32_t max_id = valhalla::midgard::Tiles<valhalla::midgard::PointLL>::MaxTileId(world_box, level->second.tiles.TileSize());

  //figure out how many digits
  //TODO: dont convert it to a string to get the length there are faster ways..
  size_t max_length = digits<uint32_t>(max_id);
  const size_t remainder = max_length % 3;
  if(remainder)
    max_length += 3 - remainder;

  //make a locale to use as a formatter for numbers
  std::ostringstream stream;
  stream.imbue(dir_locale);

  //if it starts with a zero the pow trick doesn't work
  if(graphid.level() == 0) {
    stream << static_cast<uint32_t>(std::pow(10, max_length)) + graphid.tileid() << ".gph";
    std::string suffix = stream.str();
    suffix[0] = '0';
    return suffix;
  }
  //it was something else
  stream << graphid.level() * static_cast<uint32_t>(std::pow(10, max_length)) + graphid.tileid() << ".gph";
  return stream.str();
}

}
}
