#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "costings.h"
#include "map_matching.h"


int main(int argc, char *argv[])
{
  if (argc < 5) {
    std::cout << "usage: test_map_matching (float)SIGMA_Z (float)BETA (uint)MODE (float)RADIUS" << std::endl;
    std::cout << " MODE: 0 auto; 1 auto shorter; 2 bicycle; 3 pedestrian" << std::endl;
    return 1;
  }

  float sigma_z = std::atof(argv[1]),
           beta = std::atof(argv[2]),
         radius = std::atof(argv[4]);
  int mode = std::atoi(argv[3]);

  boost::property_tree::ptree config;
  boost::property_tree::read_json("conf/valhalla.json", config);
  baldr::GraphReader graphreader(config.get_child("mjolnir.hierarchy"));

  std::shared_ptr<sif::DynamicCost> mode_costing[] = {
    nullptr, // CreateAutoCost(*config.get_child_optional("costing_options.auto")),
    nullptr, // CreateAutoShorterCost(*config.get_child_optional("costing_options.auto_shorter")),
    nullptr, // CreateBicycleCost(*config.get_child_optional("costing_options.bicycle")),
    CreateUniversalCost(*config.get_child_optional("costing_options.pedestrian"))
  };

  MapMatching mm(sigma_z, beta, graphreader, mode_costing, static_cast<sif::TravelMode>(mode));
  std::vector<Measurement> measurements;
  std::string line;
  const CandidateGridQuery grid(graphreader, 0.25/1000, 0.25/1000);
  float sq_search_radius = radius * radius;

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
