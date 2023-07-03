#include "baldr/rapidjson_utils.h"
#include <boost/property_tree/ptree.hpp>
#include <cstdint>
#include <cxxopts.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "config.h"
#include "worker.h"

#include "baldr/graphid.h"
#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "baldr/tilehierarchy.h"
#include "loki/search.h"
#include "meili/map_matcher.h"
#include "meili/map_matcher_factory.h"
#include "meili/match_result.h"
#include "midgard/encoded.h"
#include "sif/costfactory.h"
#include "thor/pathinfo.h"
#include "thor/route_matcher.h"

using namespace valhalla;
using namespace valhalla::sif;
using namespace valhalla::meili;
using namespace valhalla::baldr;
using namespace valhalla::loki;
using namespace valhalla::thor;
using namespace valhalla::midgard;

void print_edge(GraphReader& reader,
                const cost_ptr_t& costing,
                const GraphId& current_id,
                GraphId& pred_id,
                Cost& edge_total,
                Cost& trans_total,
                uint64_t& current_osmid) {
  // std::cout << "id: " << current_id << "\n";
  auto tile = reader.GetGraphTile(current_id);
  auto edge = tile->directededge(current_id);
  auto edgeinfo = tile->edgeinfo(edge);

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
    auto predinfo = tile->edgeinfo(pred_edge);
    auto node_id = pred_edge->endnode();
    auto node_tile = reader.GetGraphTile(node_id);
    auto node = node_tile->node(node_id);
    EdgeLabel pred_label(0, pred_id, pred_edge, {}, 0.0f, 0.0f, static_cast<sif::TravelMode>(0), 0,
                         {}, kInvalidRestriction, true, false, InternalTurn::kNoTurn);
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
  Cost edge_cost = costing->EdgeCost(edge, tile);
  edge_total += edge_cost;
  std::cout << "EdgeCost cost: " << edge_cost.cost << " secs: " << edge_cost.secs << "\n";
  std::cout << "------------------------\n\n";
}

void walk_edges(const std::string& shape,
                GraphReader& reader,
                const valhalla::sif::mode_costing_t& mode_costings,
                valhalla::sif::TravelMode mode) {
  auto cost = mode_costings[static_cast<uint32_t>(mode)];

  // Get shape
  std::vector<PointLL> shape_pts = decode<std::vector<PointLL>>(shape);
  if (shape_pts.size() <= 1) {
    std::cerr << "Not enough shape points to compute the path...exiting" << std::endl;
  }

  // Use the shape to form a single edge correlation at the start and end of
  // the shape (using heading).
  std::vector<valhalla::baldr::Location> locations{shape_pts.front(), shape_pts.back()};
  locations.front().heading_ = std::round(PointLL::HeadingAlongPolyline(shape_pts, 30.f));
  locations.back().heading_ = std::round(PointLL::HeadingAtEndOfPolyline(shape_pts, 30.f));

  const auto projections = Search(locations, reader, cost);
  std::vector<PathLocation> path_location;
  valhalla::Options options;

  for (const auto& ll : shape_pts) {
    auto* sll = options.mutable_shape()->Add();
    sll->mutable_ll()->set_lat(ll.lat());
    sll->mutable_ll()->set_lng(ll.lng());
    // set type to via by default
    sll->set_type(valhalla::Location::kVia);
  }
  // first and last always get type break
  if (options.shape_size()) {
    options.mutable_shape(0)->set_type(valhalla::Location::kBreak);
    options.mutable_shape(options.shape_size() - 1)->set_type(valhalla::Location::kBreak);
  }

  for (const auto& loc : locations) {
    path_location.push_back(projections.at(loc));
    PathLocation::toPBF(path_location.back(), options.mutable_locations()->Add(), reader);
  }

  std::vector<std::vector<PathInfo>> paths;
  std::vector<PathLocation> correlated;
  bool rtn = RouteMatcher::FormPath(mode_costings, mode, reader, options, paths);
  if (!rtn) {
    std::cerr << "ERROR: RouteMatcher returned false - did not match complete shape." << std::endl;
  }
  GraphId pred_id;
  GraphId current_id;
  uint64_t current_osmid = 0;
  Cost edge_total;
  Cost trans_total;
  const auto& path = paths.front();
  for (const auto& path_info : path) {
    // std::cout << "lat: " << result.lnglat.lat() << " lon: " << result.lnglat.lng() << std::endl;
    if (path_info.edgeid == current_id || path_info.edgeid == kInvalidGraphId) {
      continue;
    }

    current_id = path_info.edgeid;
    print_edge(reader, cost, current_id, pred_id, edge_total, trans_total, current_osmid);
  }

  std::cout << "+------------------------------------------------------------------------+\n";
  std::cout << "| Total Edge Cost       : " << std::setw(10) << edge_total.cost
            << "  Total Edge Secs       : " << std::setw(10) << edge_total.secs << " |\n";
  std::cout << "| Total Transition Cost : " << std::setw(10) << trans_total.cost
            << "  Total Transition Secs : " << std::setw(10) << trans_total.secs << " |\n";
  Cost total_cost = edge_total + trans_total;
  std::cout << "| Total Cost            : " << std::setw(10) << total_cost.cost
            << "  Total Secs            : " << std::setw(10) << total_cost.secs << " |\n";
  std::cout << "+------------------------------------------------------------------------+\n";
  std::cout << "\n\n";
}

// args
std::string routetype, config;
std::string json_str = "";
std::string shape = "";

// Main method for testing a single path
int main(int argc, char* argv[]) {
  try {
    // clang-format off
    cxxopts::Options options(
      "valhalla_path_comparison",
      "valhalla_path_comparison " VALHALLA_VERSION "\n\n"
      "valhalla_path_comparison is a simple command line dev tool for comparing the cost between "
      "two routes.\n"
      "Use the -j option for specifying the locations or the -s option to enter an encoded shape.\n\n");

    options.add_options()
      ("h,help", "Print this help message.")
      ("v,version", "Print the version of this software.")
      ("t,type", "Route Type: auto|bicycle|pedestrian|truck etc. Default auto.", cxxopts::value<std::string>()->default_value("auto"))
      ("s,shape", "", cxxopts::value<std::string>())
      ("j,json", R"(JSON Example: {"paths":"
        "[[{"lat":12.47,"lon":15.2},{"lat":12.46,"lon":15.21}],[{"lat":12.36,"lon":15.17},{"lat":12.37,"lon":15.18}]],"
        "costing":"bicycle","costing_options":{"bicycle":{"use_roads":0.55,"use_hills":0.1}}})", cxxopts::value<std::string>())
      ("config", "positional argument", cxxopts::value<std::string>());
    // clang-format on

    options.parse_positional({"config"});
    options.positional_help("Config file path");
    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("version")) {
      std::cout << "valhalla_path_comparison " << VALHALLA_VERSION << "\n";
      return EXIT_SUCCESS;
    }

    if (result.count("config") &&
        filesystem::is_regular_file(filesystem::path(result["config"].as<std::string>()))) {
      config = result["config"].as<std::string>();
    } else {
      std::cerr << "Configuration file is required\n\n" << options.help() << "\n\n";
      return EXIT_FAILURE;
    }

    if (result.count("json")) {
      json_str = result["json"].as<std::string>();
    } else if (result.count("shape")) {
      shape = result["shape"].as<std::string>();
    } else {
      std::cerr << "The json parameter or shape parameter was not supplied but is required.\n\n"
                << options.help() << std::endl;
      return EXIT_FAILURE;
    }
  } catch (const cxxopts::OptionException& e) {
    std::cout << "Unable to parse command line options because: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // Path Traces
  std::vector<std::vector<valhalla::baldr::Location>> paths;

  // argument checking and verification
  boost::property_tree::ptree json_ptree;
  Api request;
  ////////////////////////////////////////////////////////////////////////////
  // Process json input
  bool map_match = true;
  if (!json_str.empty()) {
    ParseApi(json_str, valhalla::Options::trace_route, request);
    std::stringstream stream(json_str);
    rapidjson::read_json(stream, json_ptree);
    try {
      for (const auto& path : json_ptree.get_child("paths")) {
        paths.push_back({});
        std::vector<valhalla::baldr::Location>& locations = paths.back();
        for (const auto& location : path.second) {
          // Get the location from the ptree
          // TODO - this was copied from the defunct Location::FromPtree
          const auto& pt = location.second;
          float lat = pt.get<float>("lat");
          if (lat < -90.0f || lat > 90.0f) {
            throw std::runtime_error("Latitude must be in the range [-90, 90] degrees");
          }
          float lon = valhalla::midgard::circular_range_clamp<float>(pt.get<float>("lon"), -180, 180);

          baldr::Location loc({lon, lat}, (pt.get<std::string>("type", "break") == "through"
                                               ? baldr::Location::StopType::THROUGH
                                               : baldr::Location::StopType::BREAK));

          loc.name_ = pt.get<std::string>("name", "");
          loc.street_ = pt.get<std::string>("street", "");

          auto date_time = pt.get_optional<std::string>("date_time");
          loc.date_time_ = date_time ? std::make_optional<std::string>(*date_time) : std::nullopt;
          auto heading = pt.get_optional<float>("heading");
          loc.heading_ = heading ? std::make_optional<float>(*heading) : std::nullopt;
          loc.heading_tolerance_ = pt.get<float>("heading_tolerance", loc.heading_tolerance_);
          loc.node_snap_tolerance_ = pt.get<float>("node_snap_tolerance", loc.node_snap_tolerance_);

          loc.min_outbound_reach_ = loc.min_inbound_reach_ =
              pt.get<unsigned int>("minimum_reachability", 50);
          loc.radius_ = pt.get<unsigned long>("radius", 0);
          locations.emplace_back(std::move(loc));
        }
      }
    } catch (...) { throw std::runtime_error("insufficiently specified required parameter 'paths'"); }
    // Parse out the type of route - this provides the costing method to use
    try {
      routetype = json_ptree.get<std::string>("costing");
    } catch (...) { throw std::runtime_error("No edge/node costing provided"); }
  } else if (!shape.empty()) {
    map_match = false;
  }

  // parse the config
  boost::property_tree::ptree pt;
  rapidjson::read_json(config.c_str(), pt);

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir"));

  if (!map_match) {
    rapidjson::Document doc;
    sif::ParseCosting(doc, "/costing_options", *request.mutable_options());
  }

  // Construct costing
  valhalla::Costing::Type costing;
  if (valhalla::Costing_Enum_Parse(routetype, &costing)) {
    request.mutable_options()->set_costing_type(costing);
  } else {
    throw std::runtime_error("No costing method found");
  }
  valhalla::sif::TravelMode mode;
  auto mode_costings = valhalla::sif::CostFactory{}.CreateModeCosting(request.options(), mode);
  auto cost_ptr = mode_costings[static_cast<uint32_t>(mode)];

  // If a shape is entered use edge walking
  if (!map_match) {
    walk_edges(shape, reader, mode_costings, mode);
    return EXIT_SUCCESS;
  }

  // If JSON is entered we do map matching
  MapMatcherFactory map_matcher_factory(pt);
  std::shared_ptr<valhalla::meili::MapMatcher> matcher(map_matcher_factory.Create(request.options()));

  uint32_t i = 0;
  for (const auto& path : paths) {
    std::cout << "==========================================================================\n";
    std::cout << "                                 PATH " << i << std::endl;
    std::cout << "==========================================================================\n\n";
    std::vector<Measurement> measurements;
    measurements.reserve(path.size());
    for (const auto& location : path) {
      measurements.emplace_back(
          Measurement{{location.latlng_.lng(), location.latlng_.lat()},
                      matcher->config().emission_cost.gps_accuracy_meters + 10,
                      matcher->config().candidate_search.search_radius_meters + 10});
    }

    auto results = matcher->OfflineMatch(measurements).front().results;

    GraphId pred_id;
    GraphId current_id;
    uint64_t current_osmid = 0;
    Cost edge_total;
    Cost trans_total;
    for (const auto& result : results) {
      // std::cout << "lat: " << result.lnglat.lat() << " lon: " << result.lnglat.lng() <<
      // std::endl;
      if (result.edgeid == current_id || result.edgeid == kInvalidGraphId) {
        continue;
      }

      current_id = result.edgeid;
      print_edge(reader, cost_ptr, current_id, pred_id, edge_total, trans_total, current_osmid);
    }
    std::cout << "+------------------------------------------------------------------------+\n";
    std::cout << "| Total Edge Cost       : " << std::setw(10) << edge_total.cost
              << "  Total Edge Secs       : " << std::setw(10) << edge_total.secs << " |\n";
    std::cout << "| Total Transition Cost : " << std::setw(10) << trans_total.cost
              << "  Total Transition Secs : " << std::setw(10) << trans_total.secs << " |\n";
    Cost total_cost = edge_total + trans_total;
    std::cout << "| Total Cost            : " << std::setw(10) << total_cost.cost
              << "  Total Secs            : " << std::setw(10) << total_cost.secs << " |\n";
    std::cout << "+------------------------------------------------------------------------+\n";
    ++i;
    std::cout << "\n\n";
  }

  return EXIT_SUCCESS;
}
