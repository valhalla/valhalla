#include "baldr/rapidjson_utils.h"
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/tokenizer.hpp>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

#include "config.h"

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "mjolnir/util.h"
#include "odin/directionsbuilder.h"
#include "odin/util.h"
#include "sif/costfactory.h"
#include "thor/bidirectional_astar.h"
#include "thor/pathalgorithm.h"
#include "thor/triplegbuilder.h"

#include <valhalla/proto/directions.pb.h>
#include <valhalla/proto/options.pb.h>
#include <valhalla/proto/trip.pb.h>

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

  bool operator<(const City& other) const {
    return country < other.country;
  }
};

// City file can be found:
// https://github.com/bahar/WorldCityLocations/blob/master/World_Cities_Location_table.csv
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
      for (const auto& t : tok) {
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

std::vector<City> GetCities(const std::string& ctry, std::vector<City>& all_cities) {
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
  str += "\"costing\":\"auto\",\"units\":\"miles\"}' --config "
         "../conf/valhalla.json\n";
  return str;
}

// Main method for testing city to city routing
int main(int argc, char* argv[]) {
  bpo::options_description options("citytest " VERSION "\n"
                                   "\n"
                                   " Usage: citytest [options]\n"
                                   "\n"
                                   "citytest is a simple command line test tool for shortest path "
                                   "routing between cities within a single country. "
                                   "\n"
                                   "Use the -c option to specify a country. "
                                   "\n"
                                   "\n");

  std::string config = "conf/planet.json";
  std::string filename = "World_Cities_Location_table.csv";

  std::string ctry;
  options.add_options()("help,h",
                        "Print this help message.")("country,c",
                                                    boost::program_options::value<std::string>(&ctry),
                                                    "Country");

  bpo::variables_map vm;
  try {
    bpo::store(bpo::parse_command_line(argc, argv, options), vm);
    if (vm.count("country")) {
      ctry = vm["country"].as<std::string>();
    } else {
      LOG_ERROR("Country was not set");
    }
  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  // Parse the config
  boost::property_tree::ptree pt;
  rapidjson::read_json(config.c_str(), pt);

  // Configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree =
      pt.get_child_optional("thor.logging");
  if (logging_subtree) {
    auto logging_config =
        valhalla::midgard::ToMap<const boost::property_tree::ptree&,
                                 std::unordered_map<std::string, std::string>>(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // Parse the input city file
  std::vector<City> all_cities = ParseCityFile(filename);
  std::cout << "City file has: " << all_cities.size() << " cities" << std::endl;

  // Get vector for just the specified country - output test route file
  std::sort(all_cities.begin(), all_cities.end());
  std::vector<City> cities = GetCities(ctry, all_cities);
  std::cout << ctry << " has: " << cities.size() << " cities" << std::endl;
  if (cities.size() > 0) {
    // Create test file
    std::string testfilename = ctry + "_routes.txt";
    std::ofstream testfile(testfilename, std::ios::out | std::ios::trunc);
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
  return EXIT_SUCCESS;
}
