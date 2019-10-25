#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>

#include "meili/map_matcher_factory.h"
#include "meili/measurement.h"

using namespace valhalla::midgard;
using namespace valhalla::meili;

template <typename istream_t>
std::vector<Measurement>
ReadMeasurements(istream_t& istream, float default_gps_accuracy, float default_search_radius) {
  std::string line;
  std::vector<Measurement> measurements;

  while (!istream.eof()) {
    std::getline(istream, line);

    if (line.empty()) {
      if (measurements.empty()) {
        continue;
      } else {
        break;
      }
    }

    // Read coordinates from the input line
    float lng, lat;
    std::stringstream stream(line);
    stream >> lng;
    stream >> lat;
    measurements.emplace_back(PointLL(lng, lat), default_gps_accuracy, default_search_radius);
  }

  return measurements;
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "usage: map_matching CONFIG" << std::endl;
    return 1;
  }

  boost::property_tree::ptree config;
  rapidjson::read_json(argv[1], config);
  const std::string modename = config.get<std::string>("meili.mode");
  valhalla::Costing costing;
  if (!valhalla::Costing_Enum_Parse(modename, &costing)) {
    throw std::runtime_error("No costing method found");
  }

  MapMatcherFactory matcher_factory(config);
  auto mapmatcher = matcher_factory.Create(costing);

  const float default_gps_accuracy = mapmatcher->config().get<float>("gps_accuracy"),
              default_search_radius = mapmatcher->config().get<float>("search_radius");

  size_t index = 0;
  for (auto measurements = ReadMeasurements(std::cin, default_gps_accuracy, default_search_radius);
       !measurements.empty();
       measurements = ReadMeasurements(std::cin, default_gps_accuracy, default_search_radius)) {

    // Offline match
    std::cout << "Sequence " << index++ << std::endl;
    auto results = mapmatcher->OfflineMatch(measurements).front().results;

    // Show results
    size_t mmt_id = 0, count = 0;
    for (const auto& result : results) {
      if (result.HasState()) {
        std::cout << mmt_id << " ";
        std::cout << result.distance_from << std::endl;
        count++;
      }
      mmt_id++;
    }

    // Summary
    std::cout << count << "/" << measurements.size() << std::endl << std::endl;

    // Clean up
    measurements.clear();
    matcher_factory.ClearFullCache();
  }

  delete mapmatcher;
  matcher_factory.ClearCache();

  return 0;
}
