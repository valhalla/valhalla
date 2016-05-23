#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "mmp/map_matching.h"

using namespace mmp;


int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cout << "usage: map_matching CONFIG" << std::endl;
    return 1;
  }

  boost::property_tree::ptree config;
  boost::property_tree::read_json(argv[1], config);

  MapMatcherFactory matcher_factory(config);
  auto matcher = matcher_factory.Create(config.get<std::string>("mm.mode"));

  std::vector<Measurement> measurements;
  std::string line;

  size_t index = 0;
  while (true) {
    std::getline(std::cin, line);
    if (std::cin.eof() || line.empty()) {

      // Offline match
      std::cout << "Sequence " << index++ << std::endl;
      const auto& results = matcher->OfflineMatch(measurements);

      // Show results
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
      std::cout << count << "/" << measurements.size() << std::endl << std::endl;

      // Clean up
      measurements.clear();
      matcher_factory.ClearFullCache();

      if (std::cin.eof()) {
        break;
      } else {
        continue;
      }
    }

    // Read coordinates from the input line
    float lng, lat;
    std::stringstream stream(line);
    stream >> lng; stream >> lat;
    measurements.emplace_back(PointLL(lng, lat));
  }

  delete matcher;
  matcher_factory.ClearCache();

  return 0;
}
