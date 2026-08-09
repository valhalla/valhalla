#include "baldr/graphid.h"
#include "baldr/rapidjson_utils.h"
#include "baldr/tilehierarchy.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <unordered_set>

namespace valhalla {
namespace baldr {

// Get the tile Id given the full path to the file.
GraphId GraphId::FromTilePath(const std::string& fname) {
  std::unordered_set<std::string::value_type> allowed{
      std::filesystem::path::preferred_separator, '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
  };
  // we require slashes
  auto pos = fname.find_last_of(std::filesystem::path::preferred_separator);
  if (pos == fname.npos) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // swallow numbers until you reach the end or a dot
  for (; pos < fname.size(); ++pos) {
    if (allowed.find(fname[pos]) == allowed.cend()) {
      break;
    }
  }
  allowed.erase(static_cast<std::string::value_type>(std::filesystem::path::preferred_separator));

  // if you didnt reach the end and it wasnt a dot then this isnt valid
  if (pos != fname.size() && fname[pos] != '.') {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // run backwards while you find an allowed char but stop if not 3 digits between slashes
  std::vector<uint32_t> digits;
  auto last = pos;
  while (--pos < last) {
    auto c = fname[pos];
    // invalid char showed up
    if (allowed.find(c) == allowed.cend()) {
      throw std::runtime_error("Invalid tile path: " + fname);
    }

    // if its the last thing or the next one is a separator thats another digit
    if (pos == 0 || fname[pos - 1] == std::filesystem::path::preferred_separator) {
      // this is not 3 or 1 digits so its wrong
      auto dist = last - pos;
      if (dist != 3 && dist != 1) {
        throw std::runtime_error("Invalid tile path: " + fname);
      }
      // we'll keep this
      auto i = atoi(fname.substr(pos, dist).c_str());
      digits.push_back(i);
      // and we'll stop if it was the level (always a single digit see GraphId)
      if (dist == 1) {
        break;
      }
      // next
      last = --pos;
    }
  }

  // if the first thing isnt a valid level bail
  if (digits.back() >= TileHierarchy::levels().size() &&
      digits.back() != TileHierarchy::GetTransitLevel().level) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // get the level info
  uint32_t level = digits.back();
  digits.pop_back();
  const auto& tile_level = level == TileHierarchy::GetTransitLevel().level
                               ? TileHierarchy::GetTransitLevel()
                               : TileHierarchy::levels()[level];

  // get the number of sub directories that we should have
  uint32_t max_id = static_cast<uint32_t>(tile_level.tiles.ncolumns() * tile_level.tiles.nrows() - 1);
  size_t parts = static_cast<size_t>(std::log10(std::max(1u, max_id))) + 1;
  if (parts % 3 != 0) {
    parts += 3 - (parts % 3);
  }
  parts /= 3;

  // bail if its the wrong number of sub dirs
  if (digits.size() != parts) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // parse the id of the tile
  int multiplier = 1;
  uint32_t id = 0;
  for (auto digit : digits) {
    id += digit * multiplier;
    multiplier *= 1000;
  }

  // if after parsing them the number is out of bounds bail
  if (id > max_id) {
    throw std::runtime_error("Invalid tile path: " + fname);
  }

  // you've passed the test enjoy your id
  return {id, level, 0};
}

// the json representation of the Id
void GraphId::json(rapidjson::writer_wrapper_t& writer) const {
  if (is_valid()) {
    writer("level", static_cast<uint64_t>(level()));
    writer("tile_id", static_cast<uint64_t>(tileid()));
    writer("id", static_cast<uint64_t>(id()));
    writer("value", value);
  }
}

// Stream output
std::ostream& operator<<(std::ostream& os, const GraphId& id) {
  return os << std::to_string(id);
}

} // namespace baldr
} // namespace valhalla
