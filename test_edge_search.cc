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
  auto costing = sif::CreatePedestrianCost(*config.get_child_optional("costing_options.pedestrian"));
  PointLL location(lon, lat);
  auto tile = reader.GetGraphTile(location);
  auto bbox = tile->BoundingBox(reader.GetTileHierarchy());
  std::cout << "Bounding box: ";
  std::cout << bbox.minx();
  std::cout << " " << bbox.maxx();
  std::cout << " "  << bbox.miny();
  std::cout << " "  << bbox.maxy() << std::endl;
  std::cout << "Number of directed edges: ";
  std::cout << tile->header()->directededgecount() << std::endl;

  CandidateQuery cq(reader);
  for (int i=0; i < 10; i++) {
    auto candidates = cq.Query(location, radius * radius, costing->GetFilter());
  }
  std::cout << "Slow query result:" << std::endl;
  auto candidates = cq.Query(location, radius * radius, costing->GetFilter());
  for (const auto& candidate : candidates) {
    std::cout << candidate.distance() << std::endl;
  }

  float cell_width = 0.25/1000,
       cell_height = 0.25/1000;
  CandidateGridQuery cgq(reader, cell_width, cell_height);
  std::cout << "Fast query result:" << std::endl;
  float sq_search_radius = radius * radius;
  auto filter = costing->GetFilter();
  for (size_t i=0; i < 10000; ++i) {
    auto candidates2 = cgq.Query(location, sq_search_radius, filter);
  }
  auto candidates2 = cgq.Query(location, radius * radius, costing->GetFilter());
  std::cout << "Number of candidates: " << candidates2.size() << std::endl;
  for (const auto& candidate : candidates2) {
    std::cout << candidate.distance() << std::endl;
  }
  return 0;
}
