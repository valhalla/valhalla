#include "argparse_utils.h"
#include "baldr/graphreader.h"
#include "midgard/logging.h"

#include <boost/property_tree/ptree.hpp>
#include <cxxopts.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

struct bitset_t {
  bitset_t(size_t size) {
    bits.resize(std::ceil(size / 64.0));
  }
  void set(const uint64_t id) {
    if (id >= bits.size() * 64) {
      throw std::runtime_error("id out of bounds");
    }
    bits[id / 64] |= static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64));
  }
  bool get(const uint64_t id) const {
    if (id >= bits.size() * 64) {
      throw std::runtime_error("id out of bounds");
    }
    return bits[id / 64] & (static_cast<uint64_t>(1) << (id % static_cast<uint64_t>(64)));
  }

protected:
  std::vector<uint64_t> bits;
};

// Main application to create a list wayids and directed edges belonging
// to ways that are drivable.
int main(int argc, char** argv) {
  const auto program = std::filesystem::path(__FILE__).stem().string();
  // args
  boost::property_tree::ptree config;
  uint64_t sample_percentage;
  std::string output;

  try {
    // clang-format off
    cxxopts::Options options(
      program,
      program + " " + VALHALLA_PRINT_VERSION + "\n\n"
      "a program that obtains the distribution of bounding circles on a tile set.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("i,inline-config", "Inline JSON config", cxxopts::value<std::string>())
      ("v,version", "Print the version of this software.")
      ("c,config", "Path to the json configuration file.", cxxopts::value<std::string>())
      ("o,output", "Path where the output files are created", cxxopts::value<std::string>(output))
      ("s,sample-size", "Sample size in % (0-100)", cxxopts::value<uint64_t>(sample_percentage)->default_value("100"));
    // clang-format on

    auto result = options.parse(argc, argv);
    if (!parse_common_args(program, options, result, &config, "mjolnir.logging"))
      return EXIT_SUCCESS;

    if (sample_percentage > 100) {
      std::cerr << "Percentage can't be larger than 100";
      return EXIT_SUCCESS;
    }

  } catch (cxxopts::exceptions::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  std::mt19937 gen(42); // fixed seed
  std::uniform_real_distribution<float> dist(0.0f, 100.0f);

  float x = dist(gen);

  // Create an unordered map of OSM ways Ids and their associated graph edges
  // get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir"));

  std::unordered_map<GraphId, uint64_t> tile_set(kMaxGraphTileId * TileHierarchy::levels().size());
  uint64_t edge_count = 0;
  uint32_t max_density = 0;
  uint32_t min_density = std::numeric_limits<uint32_t>::max();

  for (const auto& level : TileHierarchy::levels()) {
    for (uint32_t i = 0; i < level.tiles.TileCount(); ++i) {
      GraphId tile_id{i, level.level, 0};
      if (reader.DoesTileExist(tile_id)) {
        // TODO: just read the header, parsing the whole thing isnt worth it at this point
        tile_set.emplace(tile_id, edge_count);
        auto tile = reader.GetGraphTile(tile_id);
        assert(tile);
        edge_count += tile->header()->directededgecount();
        reader.Clear();
        min_density = std::min(min_density, tile->header()->density());
        max_density = std::max(max_density, tile->header()->density());
      }
    }
  }

  LOG_INFO("Approximating distribution for densities " + std::to_string(min_density) + " to " +
           std::to_string(max_density));

  // this is how we know what i've touched and what we havent
  bitset_t edge_set(edge_count);

  std::unordered_map<uint32_t, std::ofstream> streams;

  for (size_t density = min_density; density <= max_density; ++density) {
    auto& stream = streams[density];
    stream = std::ofstream(output + "/" + std::to_string(density) + ".txt");
  }

  // for each tile
  int progress = -1;
  uint64_t set = 0;
  for (const auto& tile_count_pair : tile_set) {
    // for each edge in the tile
    reader.Clear();
    auto tile = reader.GetGraphTile(tile_count_pair.first);
    assert(tile);
    auto& stream = streams[tile->header()->density()];
    for (uint32_t i = 0; i < tile->header()->directededgecount(); ++i) {
      // we've seen this one already
      if (edge_set.get(tile_count_pair.second + i)) {
        continue;
      }

      GraphId edgeid = tile_count_pair.first;
      edgeid.set_id(i);

      // make sure we dont ever look at this again
      edge_set.set(tile_count_pair.second + i);

      GraphId opp_edgeid = reader.GetOpposingEdgeId(edgeid);

      // also mark the opp edge, only if it's in the same tile so
      // we evaluate each tile-crossing shape once per tile
      if (opp_edgeid.tile_value() == edgeid.tile_value()) {
        edge_set.set(tile_count_pair.second + opp_edgeid.id());
      }

      // finally, ask the magic conch shell whether this edge will be part
      // of our sample
      if (dist(gen) > sample_percentage) {
        continue;
      }

      const auto* de = tile->directededge(edgeid);
      const auto ei = tile->edgeinfo(de);
      const auto& shape = ei.shape();
      auto bc = midgard::get_bounding_circle(shape);
      auto radius = std::to_string(std::get<1>(bc)) + "\n";
      stream.write(radius.data(), radius.size());
    }
  }

  return EXIT_SUCCESS;
}
