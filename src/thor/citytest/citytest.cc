#include <iostream>
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

#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/loki/search.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/odin/directionsbuilder.h>
#include <valhalla/odin/util.h>
#include <valhalla/proto/trippath.pb.h>
#include <valhalla/proto/tripdirections.pb.h>
#include <valhalla/proto/directions_options.pb.h>
#include <valhalla/midgard/logging.h>
#include "thor/pathalgorithm.h"
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

std::string remove_parens(const std::string& s) {
  std::string ret;
  for (auto c : s) {
    if (c != '"') {
      ret += c;
    }
  }
  return ret;
}

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
          city.country = remove_parens(t);
          break;
        case 2:
          // City
          city.city = remove_parens(t);
          break;
        case 3:
          // Latitude
          lat = std::atof(remove_parens(t).c_str());
          break;
        case 4:
          // Longitude
          lng = std::atof(remove_parens(t).c_str());
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

// Main method for testing city to city routing
int main(int argc, char *argv[]) {
  bpo::options_description options("pathtest " VERSION "\n"
  "\n"
  " Usage: citytest [options]\n"
  "\n"
  "citytest is a simple command line test tool for shortest path routing between cities within a single country. "
  "\n"
  "Use the -f option to specify the cityfile and -c option to specify a country. "
  "\n"
  "\n");

  std::string ctry = "Algeria";
  std::string config = "conf/planet.json";
  std::string filename = "World_Cities_Location_table.csv";

  // TODO - update command line processing

/*  options.add_options()("help,h", "Print this help message.")(
      "version,v", "Print the version of this software.")(
       "file,f", boost::program_options::value<std::string>(&filename),
      "World_Cities_Location_table.csv")(
      )
  // positional arguments
  ("config", bpo::value<std::string>(&config), "Valhalla configuration file");

  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);

  bpo::variables_map vm;

  try {
    bpo::store(
        bpo::command_line_parser(argc, argv).options(options).positional(
            pos_options).run(),
        vm);
    bpo::notify(vm);

  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
              << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
              << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "citytest " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

/*

  // argument checking and verification
  if (vm.count("json") == 0) {
    for (auto arg : std::vector<std::string> { "origin", "destination", "type",
        "config" }) {
      if (vm.count(arg) == 0) {
        std::cerr
            << "The <"
            << arg
            << "> argument was not provided, but is mandatory when json is not provided\n\n";
        std::cerr << options << "\n";
        return EXIT_FAILURE;
      }
    }
    originloc = Location::FromCsv(origin);
    destloc = Location::FromCsv(destination);
  } else {
    std::stringstream stream;
    stream << json;
    boost::property_tree::ptree json_ptree;
    boost::property_tree::read_json(stream, json_ptree);
    std::vector<Location> locations;
    try {
      for (const auto& location : json_ptree.get_child("locations"))
        locations.emplace_back(std::move(Location::FromPtree(location.second)));
      if (locations.size() < 2)
        throw;
    } catch (...) {
      throw std::runtime_error(
          "insufficiently specified required parameter 'locations'");
    }

    // Parse out the type of route - this provides the costing method to use
    std::string costing;
    try {
      routetype = json_ptree.get<std::string>("costing");
    } catch (...) {
      throw std::runtime_error("No edge/node costing provided");
    }

    // Grab the directions options, if they exist
    auto directions_options_ptree_ptr = json_ptree.get_child_optional(
        "directions_options");
    if (directions_options_ptree_ptr) {
      directions_options = valhalla::odin::GetDirectionsOptions(
          *directions_options_ptree_ptr);
    }

  }
*/
  // parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  //configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt
      .get_child_optional("thor.logging");
  if (logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<
        const boost::property_tree::ptree&,
        std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  // Construct costing
//  boost::property_tree::ptree costing = pt.get_child("costing");
//  for (const auto cm : costing) {
//    std::cout << "Costing method: " << cm.first << std::endl;
//  }

  std::string routetype = "auto";

  // Any good way to ties these into the config?
  CostFactory<DynamicCost> factory;
  factory.Register("auto", CreateAutoCost);
  factory.Register("auto-shorter", CreateAutoShorterCost);
  factory.Register("bicycle", CreateBicycleCost);
  factory.Register("pedestrian", CreatePedestrianCost);

  // Figure out the route type
  for (auto & c : routetype)
    c = std::tolower(c);

  LOG_INFO("routetype: " + routetype);

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir.hierarchy"));

  // Parse the input city file
  std::vector<City> all_cities = ParseCityFile(filename);

  std::cout << "City file has: " << all_cities.size() << " cities" << std::endl;

  // Get vector for just the country(s) specified
  std::sort(all_cities.begin(), all_cities.end());
  std::vector<City> cities = GetCities(ctry, all_cities);

  std::cout << ctry << " has: " << cities.size() << " cities" << std::endl;

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
      std::shared_ptr<DynamicCost> cost = factory.Create(
           routetype, pt.get_child("costing_options." + routetype));

      Location originloc(cities[l0].latlng);
      Location destloc(cities[l1].latlng);
//LOG_INFO("Origin LL= " + std::to_string(cities[l0].latlng.lat()) + "," + std::to_string(cities[l0].latlng.lng()));
      // Use Loki to get location information
      PathLocation origin = Search(originloc, reader, cost->GetFilter());
//LOG_INFO("Dest LL= " + std::to_string(cities[l1].latlng.lat()) + "," + std::to_string(cities[l1].latlng.lng()));
      PathLocation dest   = Search(destloc, reader, cost->GetFilter());

      uint32_t np = 0;
      PathAlgorithm pathalgorithm;
      std::vector<PathInfo> pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
      if (pathedges.size() == 0) {
        // 2nd pass - increase hierarchy limits, 3rd pass disable highway
        // transitions
        if (cost->AllowMultiPass()) {
          pathalgorithm.Clear();
          cost->RelaxHierarchyLimits(16.0f);
          pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
          np++;
          if (pathedges.size() == 0) {
            cost->DisableHighwayTransitions();
            pathedges = pathalgorithm.GetBestPath(origin, dest, reader, cost);
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

