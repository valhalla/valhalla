#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/sif/pedestriancost.h>
#include <valhalla/baldr/location.h>

#include "edge_search.h"

using namespace valhalla;

int main(int argc, char *argv[])
{
  if (argc < 4) {
    std::cout << "usage: test_edge_search LON LAT RADIUS" << std::endl;
    return 1;
  }
  float lon =  std::atof(argv[1]),
        lat = std::atof(argv[2]),
     radius = std::atof(argv[3]);

  boost::property_tree::ptree config;
  boost::property_tree::read_json("conf/valhalla.json", config);
  valhalla::baldr::GraphReader reader(config.get_child("mjolnir.hierarchy"));
  float sq_radius = radius * radius;
  auto costing = sif::CreatePedestrianCost(*config.get_child_optional("costing_options.pedestrian"));
  Location location{{lon, lat}, baldr::Location::StopType::BREAK};
  for (int i=0; i < 10; i++) {
    auto candidates = EdgeSearch(location, reader, costing->GetFilter(), sq_radius);
  }
  auto candidates = EdgeSearch(location, reader, costing->GetFilter(), sq_radius);
  for (const auto& candidate : candidates) {
    std::cout << candidate.distance() << std::endl;
  }
  return 0;
}
