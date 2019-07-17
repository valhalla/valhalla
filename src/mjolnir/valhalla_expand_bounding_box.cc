#include "baldr/graphreader.h"
#include "baldr/predictedspeeds.h"
#include "midgard/logging.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/util.h"
#include <cmath>
#include <cstdint>

#include "baldr/rapidjson_utils.h"
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/binary_from_base64.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>

#include <deque>
#include <future>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "config.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vj = valhalla::mjolnir;

namespace bpo = boost::program_options;
namespace bpt = boost::property_tree;
namespace bfs = boost::filesystem;

int main(int argc, char** argv) {
  std::string config, bbox;
  std::string inline_config;
  boost::filesystem::path config_file_path;

  unsigned int num_threads = 1;

  bpo::options_description options("valhalla_expand_bounding_box " VALHALLA_VERSION "\n"
                                   "\n"
                                   " Usage: valhalla_expand_bounding_box [options]\n"
                                   "\n"
                                   "Finds all the nodes in the bounding box and then expands "
                                   "the bounding box by the shape of the edges that leave the nodes."
                                   "\n"
                                   "\n");

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")
                                                              ("config,c",
                                   boost::program_options::value<boost::filesystem::path>(
                                       &config_file_path),
                                   "Path to the json configuration file.")("inline-config,i",
                                                                           boost::program_options::
                                                                               value<std::string>(
                                                                                   &inline_config),
                                                                           "Inline json config.")
      // positional arguments
      ("bounding-box,b", bpo::value<std::string>(&bbox), "Bounding box to expand. The format is lower "
          "left lng/lat and upper right lng/lat or min_x,min_y,max_x,max_y");

  bpo::positional_options_description pos_options;
  pos_options.add("bounding-box", 1);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
    bpo::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "valhalla_expand_bounding_box " << VALHALLA_VERSION << "\n";
    return EXIT_SUCCESS;
  }

  if (!vm.count("bounding-box")) {
    std::cout << "You must provide a bounding box to expand.\n";
    return EXIT_FAILURE;
  }

  // Read the config file
  boost::property_tree::ptree pt;
  if (vm.count("inline-config")) {
    std::stringstream ss;
    ss << inline_config;
    rapidjson::read_json(ss, pt);
  } else if (vm.count("config") && boost::filesystem::is_regular_file(config_file_path)) {
    rapidjson::read_json(config_file_path.string(), pt);
  } else {
    std::cerr << "Configuration is required\n\n" << options << "\n\n";
    return EXIT_FAILURE;
  }

  // configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("mjolnir.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  std::stringstream ss(bbox);
  std::vector<float> result;

  while(ss.good()) {
      std::string substr;
      getline( ss, substr,',');
      result.push_back(std::stof(substr));
  }

  if (result.size() != 4) {
    std::cout << "You must provide a valid bounding box to expand.\n";
    return EXIT_FAILURE;
  }

  AABB2<PointLL> bb{{result[0], result[1]}, {result[2], result[3]}};
  AABB2<PointLL> expanded_bb = bb;

  const auto& ids = vb::TileHierarchy::GetGraphIds(bb);
  GraphReader reader(pt.get_child("mjolnir"));
  // Iterate through the tiles
  for (const auto& tile_id : ids) {
    if (reader.OverCommitted()) {
      reader.Clear();
    }
    const GraphTile* tile = reader.GetGraphTile(tile_id);
    for (uint32_t i = 0; i < tile->header()->nodecount(); i++) {
      const NodeInfo* node = tile->node(i);
      if (bb.Contains(node->latlng(tile->header()->base_ll()))) {
        // Iterate through outbound driveable edges.
        const DirectedEdge* diredge = tile->directededge(node->edge_index());
        for (uint32_t i = 0; i < node->edge_count(); i++, diredge++) {
          // Skip opposing directed edge and any edge that is not a road. Skip any
          // edges that are not driveable outbound.
          if (!(diredge->forwardaccess() & kAutoAccess)) {
            continue;
          }

          auto shape = tile->edgeinfo(diredge->edgeinfo_offset()).shape();
          for (const auto& s : shape) {
            expanded_bb.Expand(s);
          }
        }
      }
    }
  }
  std::cout << std::to_string(expanded_bb.minx()) << "," << expanded_bb.miny() << "," << expanded_bb.maxx() << "," << expanded_bb.maxy() << std::endl;

  return EXIT_SUCCESS;
}
