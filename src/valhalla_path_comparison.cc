#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

#include "config.h"

#include "baldr/graphreader.h"
#include "baldr/tilehierarchy.h"
#include "baldr/pathlocation.h"
#include "baldr/graphid.h"
#include "sif/costfactory.h"
#include "meili/map_matcher.h"
#include "meili/map_matcher_factory.h"
#include "meili/match_result.h"

using namespace valhalla::sif;
using namespace valhalla::meili;
using namespace valhalla::baldr;

namespace bpo = boost::program_options;

// Main method for testing a single path
int main(int argc, char *argv[]) {
  bpo::options_description options("valhalla_path_comparison " VERSION "\n"
  "\n"
  " Usage: valhalla_path_comparison [options]\n"
  "\n"
  "valhalla_path_comparison is a simple command line dev tool for comparing the cost between two routes. "
  "\n"
  "Use the -j option for specifying the locations. "
  "\n"
  "\n");

  std::string routetype, json, config;

  options.add_options()("help,h", "Print this help message.")(
      "version,v", "Print the version of this software.")(
      "type,t", boost::program_options::value<std::string>(&routetype),
      "Route Type: auto|bicycle|pedestrian|auto-shorter")(
      "json,j",
      boost::program_options::value<std::string>(&json),
      R"(JSON Example: {"paths":[[{"lat":12.47,"lon":15.2},{"lat":12.46,"lon":15.21}],[{"lat":12.36,"lon":15.17},{"lat":12.37,"lon":15.18}]],"costing":"bicycle","costing_options":{"bicycle":{"use_roads":0.55,"use_hills":0.1}}})")
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
    std::cout << "valhalla_path_comparison " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // Path Traces
  std::vector<std::vector<valhalla::baldr::Location>> paths;

  // argument checking and verification
  boost::property_tree::ptree json_ptree;
  ////////////////////////////////////////////////////////////////////////////
  // Process json input
  if (vm.count("json")) {
    std::stringstream stream(json);
    boost::property_tree::read_json(stream, json_ptree);
    try {
      for (const auto& path : json_ptree.get_child("paths")) {
        paths.push_back({});
        std::vector<valhalla::baldr::Location>& locations = paths.back();
        for (const auto& location: path.second) {
          locations.emplace_back(std::move(valhalla::baldr::Location::FromPtree(location.second)));
        }
      }
    } catch (...) {
      throw std::runtime_error(
          "insufficiently specified required parameter 'paths'");
    }
    // Parse out the type of route - this provides the costing method to use
    try {
      routetype = json_ptree.get<std::string>("costing");
    } catch (...) {
      throw std::runtime_error("No edge/node costing provided");
    }
  } else {
    std::cerr << "The json parameter was not supplied but is required.\n\n" << options << std::endl;
    return EXIT_FAILURE;
  }

  //parse the config
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config.c_str(), pt);

  // Construct costing
  CostFactory<DynamicCost> factory;
  factory.Register("auto", CreateAutoCost);
  factory.Register("bicycle", CreateBicycleCost);
  factory.Register("pedestrian", CreatePedestrianCost);
  factory.Register("motor_scooter", CreateMotorScooterCost);
  std::string method_options = "costing_options." + routetype;
  auto costing_options = json_ptree.get_child(method_options, {});
  cost_ptr_t costing = factory.Create(routetype, costing_options);

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir"));

  MapMatcherFactory map_matcher_factory (pt);
  std::shared_ptr<valhalla::meili::MapMatcher> matcher (map_matcher_factory.Create(routetype, pt));

  uint32_t i = 0;
  for (const auto& path : paths) {
    std::cout << "==========================================================================\n";
    std::cout << "                                 PATH " << i << std::endl;
    std::cout << "==========================================================================\n\n";
    std::vector<Measurement> measurements;
    for (const auto& location : path) {
      measurements.emplace_back (Measurement{
          {location.latlng_.lng(),location.latlng_.lat()},
          matcher->config().get<float>("gps_accuracy") + 10,
          matcher->config().get<float>("search_radius") + 10
      });
    }

    auto results = matcher->OfflineMatch(measurements).front().results;

    GraphId pred_id;
    GraphId current_id;
    uint64_t current_osmid = 0;
    Cost edge_total;
    Cost trans_total;
    for (const auto& result : results) {
      //std::cout << "lat: " << result.lnglat.lat() << " lon: " << result.lnglat.lng() << std::endl;
      if (result.edgeid == current_id || result.edgeid == kInvalidGraphId) {
        continue;
      }

      current_id = result.edgeid;
      //std::cout << "id: " << current_id << "\n";
      auto tile = reader.GetGraphTile(current_id);
      auto edge = tile->directededge(current_id);
      auto edgeinfo = tile->edgeinfo(edge->edgeinfo_offset());

      if (edgeinfo.wayid() != current_osmid) {
        current_osmid = edgeinfo.wayid();
        std::string name = edgeinfo.GetNames().size() == 0 ? "unnamed" : edgeinfo.GetNames()[0];
        std::cout << "+++++++++++++++++++++++++++++++++++++\n";
        std::cout << "wayid: " << current_osmid << std::endl;
        std::cout << "name: " << name << std::endl;
        std::cout << "+++++++++++++++++++++++++++++++++++++\n\n";
      }

      if (pred_id != kInvalidGraphId) {
        auto pred_tile = reader.GetGraphTile(pred_id);
        auto pred_edge = pred_tile->directededge(pred_id);
        auto predinfo = tile->edgeinfo(pred_edge->edgeinfo_offset());
        auto node_id = pred_edge->endnode();
        auto node_tile = reader.GetGraphTile (node_id);
        auto node = node_tile->node(node_id);
        EdgeLabel pred_label (0, pred_id, pred_edge, {}, 0.0f, 0.0f, static_cast<TravelMode>(0), 0);
        std::cout << "-------Transition-------\n";
        std::cout << "Pred GraphId: " << pred_id << std::endl;
        Cost trans_cost = costing->TransitionCost(edge, node, pred_label);
        trans_total += trans_cost;
        std::cout << "TransitionCost cost: " << trans_cost.cost;
        std::cout << " secs: " << trans_cost.secs << "\n";
        std::cout << "------------------------\n\n";
      }
      pred_id = current_id;

      std::cout << "----------Edge----------\n";
      std::cout << "Edge GraphId: " << current_id << std::endl;
      std::cout << "Edge length: " << edge->length() << std::endl;
      Cost edge_cost = costing->EdgeCost(edge);
      edge_total += edge_cost;
      std::cout << "EdgeCost cost: " << edge_cost.cost << " secs: " << edge_cost.secs << "\n";
      std::cout << "------------------------\n\n";
    }
    std::cout << "+------------------------------------------------------------------------+\n";
    std::cout << "| Total Edge Cost       : " << std::setw(10) << edge_total.cost << "  Total Edge Secs       : " << std::setw(10) << edge_total.secs << " |\n";
    std::cout << "| Total Transition Cost : " << std::setw(10) << trans_total.cost << "  Total Transition Secs : " << std::setw(10) << trans_total.secs << " |\n";
    Cost total_cost = edge_total + trans_total;
    std::cout << "| Total Cost            : " << std::setw(10) << total_cost.cost << "  Total Secs            : " << std::setw(10) << total_cost.secs << " |\n";
    std::cout << "+------------------------------------------------------------------------+\n";
    ++i;
    std::cout << "\n\n";
  }

  return EXIT_SUCCESS;
}

