#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "costings.h"
#include "map_matching.h"


int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cout << "usage: map_matching CONFIG" << std::endl;
    return 1;
  }

  boost::property_tree::ptree config;
  boost::property_tree::read_json(argv[1], config);
  baldr::GraphReader graphreader(config.get_child("mjolnir.hierarchy"));

  std::shared_ptr<sif::DynamicCost> mode_costing[64];
  const auto costing = CreateUniversalCost(config.get_child("costing_options.multimodal"));
  mode_costing[static_cast<size_t>(costing->travelmode())] = costing;
  auto mm_config = config.get_child("mm");

  MapMatching mm(graphreader, mode_costing, costing->travelmode(), mm_config);
  const CandidateGridQuery grid(graphreader,
                                local_tile_size(graphreader)/config.get<size_t>("grid.size"),
                                local_tile_size(graphreader)/config.get<size_t>("grid.size"));
  const auto radius = mm_config.get<float>("search_radius");
  const auto sq_search_radius = radius * radius;

  std::vector<Measurement> measurements;
  std::string line;

  size_t index = 0;
  while (true) {
    std::getline(std::cin, line);
    if (std::cin.eof() || line.empty()) {
      std::cout << "============================" << std::endl;
      std::cout << index++ << " id: " << std::endl;

      const auto& results = OfflineMatch(mm, grid, measurements, sq_search_radius);
      size_t mmt_id = 0, count = 0;
      for (const auto& result : results) {
        const auto state = result.state();
        if (state) {
          std::cout << mmt_id << " ";
          std::cout << state->id() << " ";
          std::cout << state->candidate().distance() << std::endl;
          count++;
        }
        mmt_id++;
      }

      // Summary
      std::cout << count << "/" << measurements.size() << std::endl;

      // Clean up
      measurements.clear();

      if (std::cin.eof()) {
        break;
      } else {
        continue;
      }
    }

    // Load coordinates from the input line
    float lng, lat;
    std::stringstream stream(line);
    stream >> lng; stream >> lat;
    measurements.emplace_back(PointLL(lng, lat));
  }

  return 0;
}
