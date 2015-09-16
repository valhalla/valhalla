#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// #include <valhalla/sif/autocost.h>
// #include <valhalla/sif/bicyclecost.h>
// #include <valhalla/sif/pedestriancost.h>
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
           mode = std::atoi(argv[3]),
         radius = std::atof(argv[4]);

  boost::property_tree::ptree config;
  boost::property_tree::read_json("conf/valhalla.json", config);
  baldr::GraphReader graphreader(config.get_child("mjolnir.hierarchy"));

  std::shared_ptr<sif::DynamicCost> mode_costing[] = {
    nullptr, // CreateAutoCost(*config.get_child_optional("costing_options.auto")),
    nullptr, // CreateAutoShorterCost(*config.get_child_optional("costing_options.auto_shorter")),
    nullptr, // CreateBicycleCost(*config.get_child_optional("costing_options.bicycle")),
    CreatePedestrianCost(*config.get_child_optional("costing_options.pedestrian"))
  };

  MapMatching mm(sigma_z, beta, graphreader, mode_costing, static_cast<sif::TravelMode>(mode));
  std::vector<Measurement> measurements;
  MeasurementId mmt_id = 0;
  while (!std::cin.eof()) {
    float lng, lat;
    std::cin >> lng;
    std::cin >> lat;
    measurements.push_back({mmt_id++, PointLL(lng, lat)});
  }

  auto path = OfflineMatch(mm, measurements, radius);

  for (const auto candidate_ptr : path) {
    if (candidate_ptr) {
      auto& candidate = candidate_ptr->candidate();
      auto measurement = mm.measurement(candidate_ptr->time());
      std::cout << measurement.id << " ";
      std::cout << candidate_ptr->id() << " ";
      std::cout << candidate.distance() << std::endl;
    }
  }
  return 0;
}
