#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>

#include "config.h"

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "loki/search.h"
#include "sif/costfactory.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "mjolnir/util.h"
#include "proto/trippath.pb.h"
#include "proto/tripdirections.pb.h"
#include "proto/directions_options.pb.h"
#include "midgard/logging.h"
#include "thor/pathalgorithm.h"
#include "thor/bidirectional_astar.h"
#include "thor/trippathbuilder.h"

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::odin;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace bpo = boost::program_options;

struct City {
  std::string country;
  std::string city;
  PointLL latlng;

  bool operator < (const City& other) const {
    return country < other.country;
  }
};

std::vector<City> ParseCityFile(const std::string& filename) {
  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{";"};
  std::vector<City> cities;

  // Open file
  float lat, lng;
  std::string line;
  std::ifstream file(filename);
  if (file.is_open()) {
    while (getline(file, line)) {
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      City city{};
      for (const auto &t : tok) {
        switch (field_num) {
        case 0:
        case 5:
          break;
        case 1:
          // Country (remove first and last char)
          city.country = remove_double_quotes(t);
          break;
        case 2:
          // City
          city.city = remove_double_quotes(t);
          break;
        case 3:
          // Latitude
          lat = std::atof(remove_double_quotes(t).c_str());
          break;
        case 4:
          // Longitude
          lng = std::atof(remove_double_quotes(t).c_str());
          break;
        }
        field_num++;
      }
      city.latlng.Set(lng, lat);
      cities.emplace_back(std::move(city));
    }
    file.close();
  } else {
    std::cout << "City file: " << filename << " not found" << std::endl;
  }

  return cities;
}

std::vector<City> GetCities(const std::string& ctry,
                            std::vector<City>& all_cities) {
  std::vector<City> cities;
  City test_ctry;
  test_ctry.country = ctry;
  auto p = std::equal_range(all_cities.begin(), all_cities.end(), test_ctry);
  for (auto i = p.first; i != p.second; ++i) {
    cities.push_back(*i);
  }
  return cities;
}

// Get the JSON request string for an auto route
std::string GetJSONRequest(const Location& originloc, const Location& destloc) {
  // Add origin location
  std::string str = "-j '{\"locations\":[{\"lat\":";
  str += std::to_string(originloc.latlng_.lat());
  str += ",\"lon\":";
  str += std::to_string(originloc.latlng_.lng());
  str += ",\"city\":\"";
  str += originloc.city_;
  str += "\"},";

  // Add destination location
  str += "{\"lat\":";
  str += std::to_string(destloc.latlng_.lat());
  str += ",\"lon\":";
  str += std::to_string(destloc.latlng_.lng());
  str += ",\"city\":\"";
  str += destloc.city_;
  str += "\"}],";

  // Add costing and return string
  str += "\"costing\":\"auto\",\"directions_options\":{\"units\":\"miles\"}}' --config ../conf/valhalla.json\n";
  return str;
}

// Main method for testing city to city routing
int main(int argc, char *argv[]) {
  bpo::options_description options("citytest " VERSION "\n"
  "\n"
  " Usage: citytest [options]\n"
  "\n"
  "citytest is a simple command line test tool for shortest path routing between cities within a single country. "
  "\n"
  "Use the -c option to specify a country. "
  "\n"
  "\n");

  std::string config = "conf/planet.json";
  std::string filename = "World_Cities_Location_table.csv";

  std::string ctry;
  options.add_options()
      ("help,h", "Print this help message.")
      ("country,c", boost::program_options::value<std::string>(&ctry), "Country");

  bpo::variables_map vm;
  try {
    bpo::store(bpo::parse_command_line(argc, argv, options), vm);
    if (vm.count("country")) {
      ctry = vm["country"].as<std::string>();
    } else {
      LOG_ERROR("Country was not set");
    }
  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
              << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
              << "\n";
    return EXIT_FAILURE;
  }

  // Parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  // Configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
            pt.get_child_optional("thor.logging");
  if (logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<
        const boost::property_tree::ptree&,
        std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // Parse the input city file
  std::vector<City> all_cities = ParseCityFile(filename);
  std::cout << "City file has: " << all_cities.size() << " cities" << std::endl;

  // Get vector for just the specified country - output test route file
  std::sort(all_cities.begin(), all_cities.end());
  std::vector<City> cities = GetCities(ctry, all_cities);
  std::cout << ctry << " has: " << cities.size() << " cities" << std::endl;
  if (cities.size()> 0) {
    // Create test file
    std::string testfilename = ctry + "_routes.txt";
    std::ofstream testfile(testfilename, std::ios::out | std::ios::trunc );
    if (testfile.is_open()) {
      std::cout << "Open " << testfilename << std::endl;
      for (uint32_t l0 = 0; l0 < cities.size() - 1; l0++) {
        for (uint32_t l1 = l0 + 1; l1 < cities.size(); l1++) {
          Location originloc(cities[l0].latlng);
          originloc.city_ = cities[l0].city;
          Location destloc(cities[l1].latlng);
          destloc.city_ = cities[l1].city;
          testfile << GetJSONRequest(originloc, destloc);
        }
      }
      testfile.close();
    } else {
      LOG_ERROR("Failed to open output test file");
      return EXIT_FAILURE;
    }
  } else {
    LOG_ERROR("No cities in " + ctry + " - no test file created");
    return EXIT_FAILURE;
  }

  // Exit here or continue and run routes?
//  return EXIT_SUCCESS;

  CostFactory<DynamicCost> factory;
  factory.Register("auto", CreateAutoCost);
  factory.Register("auto_shorter", CreateAutoShorterCost);
  factory.Register("bus", CreateBusCost);
  factory.Register("bicycle", CreateBicycleCost);
  factory.Register("pedestrian", CreatePedestrianCost);

  // Figure out the route type
  std::string routetype = "auto";
  for (auto & c : routetype)
    c = std::tolower(c);

  LOG_INFO("routetype: " + routetype);

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir.hierarchy"));

  // Run routes
  uint32_t error_count = 0;
  uint32_t success_count = 0;
  uint32_t npasses[3] = {};
  auto t1 = std::chrono::high_resolution_clock::now();
  for (uint32_t l0 = 0; l0 < cities.size() - 1; l0++) {
    for (uint32_t l1 = l0 + 1; l1 < cities.size(); l1++) {
      // Get the costing method. We do this each time since prior route may
      // have changed hierarchy_limits. Also, this simulates how the service
      // works
      cost_ptr_t mode_costing[4];
      cost_ptr_t cost = factory.Create(
          routetype, pt.get_child("costing_options." + routetype));
      TravelMode mode = cost->travelmode();
      mode_costing[static_cast<uint32_t>(mode)] = cost;


      Location originloc(cities[l0].latlng);
      Location destloc(cities[l1].latlng);

      // Use Loki to get location information
      PathLocation origin = Search(originloc, reader, cost->GetEdgeFilter(), cost->GetNodeFilter());
      PathLocation dest   = Search(destloc, reader, cost->GetEdgeFilter(), cost->GetNodeFilter());

      // TODO - maybe later use different path algorithms
      uint32_t np = 0;
      PathAlgorithm pathalgorithm;
      std::vector<PathInfo> pathedges = pathalgorithm.GetBestPath(origin, dest, reader, mode_costing, mode);
      if (pathedges.size() == 0) {
        // 2nd pass - increase hierarchy limits, 3rd pass disable highway
        // transitions
        if (cost->AllowMultiPass()) {
          pathalgorithm.Clear();
          cost->RelaxHierarchyLimits(16.0f, 4.0f);
          pathedges = pathalgorithm.GetBestPath(origin, dest, reader, mode_costing, mode);
          np++;
          if (pathedges.size() == 0) {
            pathalgorithm.Clear();
            cost->DisableHighwayTransitions();
            pathedges = pathalgorithm.GetBestPath(origin, dest, reader, mode_costing, mode);
            np++;
          }
        }
      }

      if (pathedges.size() == 0) {
        error_count++;
      } else {
        success_count++;
        npasses[np]++;
      }

      // TODO - perhaps walk the edges to find total length?
    }
  }
  LOG_INFO(std::to_string(success_count) + " out of " +
           std::to_string(success_count+error_count) + " succeeded");
  LOG_INFO("Success on first pass: " + std::to_string(npasses[0]));
  LOG_INFO("Success on second pass: " + std::to_string(npasses[1]));
  LOG_INFO("Success on third pass: " + std::to_string(npasses[2]));
  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t msecs = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
  float secs = msecs * 0.001f;
  LOG_INFO("Time = " + std::to_string(secs) + " secs");

  return EXIT_SUCCESS;
}

