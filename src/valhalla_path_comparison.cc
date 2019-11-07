#include "baldr/rapidjson_utils.h"
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cstdint>
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

namespace bpo = boost::program_options;

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
    auto node_tile = reader.GetGraphTile(node_id);
    auto node = node_tile->node(node_id);
    EdgeLabel pred_label(0, pred_id, pred_edge, {}, 0.0f, 0.0f, static_cast<TravelMode>(0), 0);
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

void walk_edges(const std::string& shape, GraphReader& reader, cost_ptr_t cost_ptr) {
  TravelMode mode = cost_ptr->travel_mode();
  cost_ptr_t mode_costing[10];
  mode_costing[static_cast<uint32_t>(mode)] = cost_ptr;

  // Decode the shape
  std::vector<PointLL> shape_pts = decode<std::vector<PointLL>>(shape);
  if (shape_pts.size() <= 1) {
    std::cerr << "Not enough shape points to compute the path...exiting" << std::endl;
  }

  std::vector<Measurement> measurements;
  measurements.reserve(shape_pts.size());
  for (const auto& ll : shape_pts) {
    measurements.emplace_back(Measurement{ll, 10, 10});
  }

  // Add a location for the origin (first shape point) and destination (last
  // shape point)
  std::vector<baldr::Location> locations;
  locations.push_back({shape_pts.front()});
  locations.push_back({shape_pts.back()});
  const auto projections = Search(locations, reader, cost_ptr.get());
  std::vector<PathLocation> path_location;
  valhalla::Options options;
  for (const auto& loc : locations) {
    try {
      path_location.push_back(projections.at(loc));
      PathLocation::toPBF(path_location.back(), options.mutable_locations()->Add(), reader);
    } catch (...) { return; }
  }

  std::vector<PathInfo> path_infos;
  std::vector<PathLocation> correlated;
  bool rtn = RouteMatcher::FormPath(mode_costing, mode, reader, measurements, options, path_infos);
  if (!rtn) {
    std::cerr << "ERROR: RouteMatcher returned false - did not match complete shape." << std::endl;
  }
  GraphId pred_id;
  GraphId current_id;
  uint64_t current_osmid = 0;
  Cost edge_total;
  Cost trans_total;
  for (const auto& path_info : path_infos) {
    // std::cout << "lat: " << result.lnglat.lat() << " lon: " << result.lnglat.lng() << std::endl;
    if (path_info.edgeid == current_id || path_info.edgeid == kInvalidGraphId) {
      continue;
    }

    current_id = path_info.edgeid;
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
  std::cout << "\n\n";
}

// Main method for testing a single path
int main(int argc, char* argv[]) {
  bpo::options_description options(
      "valhalla_path_comparison " VALHALLA_VERSION "\n"
      "\n"
      " Usage: valhalla_path_comparison [options]\n"
      "\n"
      "valhalla_path_comparison is a simple command line dev tool for comparing the cost between "
      "two routes. "
      "\n"
      "Use the -j option for specifying the locations or the -s option to enter an encoded shape."
      "\n"
      "\n");

  std::string routetype, json, shape, config;

  options.add_options()("help,h", "Print this help message.")("version,v",
                                                              "Print the version of this software.")(
      "type,t", boost::program_options::value<std::string>(&routetype),
      "Route Type: auto|bicycle|pedestrian|auto-shorter")("shape,s",
                                                          boost::program_options::value<std::string>(
                                                              &shape),
                                                          "")(
      "json,j", boost::program_options::value<std::string>(&json),
      R"(JSON Example: {"paths":[[{"lat":12.47,"lon":15.2},{"lat":12.46,"lon":15.21}],[{"lat":12.36,"lon":15.17},{"lat":12.37,"lon":15.18}]],"costing":"bicycle","costing_options":{"bicycle":{"use_roads":0.55,"use_hills":0.1}}})")
      // positional arguments
      ("config", bpo::value<std::string>(&config), "Valhalla configuration file");

  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);

  bpo::variables_map vm;

  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(),
               vm);
    bpo::notify(vm);

  } catch (std::exception& e) {
    std::cerr << "Unable to parse command line options because: " << e.what() << "\n"
              << "This is a bug, please report it at " PACKAGE_BUGREPORT << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "valhalla_path_comparison " << VALHALLA_VERSION << "\n";
    return EXIT_SUCCESS;
  }

  // Path Traces
  std::vector<std::vector<valhalla::baldr::Location>> paths;

  // argument checking and verification
  boost::property_tree::ptree json_ptree;
  Api request;
  ////////////////////////////////////////////////////////////////////////////
  // Process json input
  bool map_match = true;
  if (vm.count("json")) {
    ParseApi(json, valhalla::Options::trace_route, request);
    std::stringstream stream(json);
    rapidjson::read_json(stream, json_ptree);
    try {
      for (const auto& path : json_ptree.get_child("paths")) {
        paths.push_back({});
        std::vector<valhalla::baldr::Location>& locations = paths.back();
        for (const auto& location : path.second) {
          // Get the location from the ptree
          // TODO - this was copied from the defunct Location::FromPtree
          const auto& pt = path.second;
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
          loc.city_ = pt.get<std::string>("city", "");
          loc.state_ = pt.get<std::string>("state", "");
          loc.zip_ = pt.get<std::string>("postal_code", "");
          loc.country_ = pt.get<std::string>("country", "");

          loc.date_time_ = pt.get_optional<std::string>("date_time");
          loc.heading_ = pt.get_optional<float>("heading");
          loc.heading_tolerance_ = pt.get<float>("heading_tolerance", loc.heading_tolerance_);
          loc.node_snap_tolerance_ = pt.get<float>("node_snap_tolerance", loc.node_snap_tolerance_);
          loc.way_id_ = pt.get_optional<long double>("way_id");

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
  } else if (vm.count("shape")) {
    map_match = false;
  } else {
    std::cerr << "The json parameter or shape parameter was not supplied but is required.\n\n"
              << options << std::endl;
    return EXIT_FAILURE;
  }

  // parse the config
  boost::property_tree::ptree pt;
  rapidjson::read_json(config.c_str(), pt);

  // Get something we can use to fetch tiles
  valhalla::baldr::GraphReader reader(pt.get_child("mjolnir"));

  // Construct costing
  CostFactory<DynamicCost> factory;
  factory.RegisterStandardCostingModels();
  valhalla::Costing costing;
  if (valhalla::Costing_Enum_Parse(routetype, &costing)) {
    request.mutable_options()->set_costing(costing);
  } else {
    throw std::runtime_error("No costing method found");
  }
  cost_ptr_t cost_ptr = factory.Create(costing, request.options());

  // If a shape is entered use edge walking
  if (!map_match) {
    walk_edges(shape, reader, cost_ptr);
    return EXIT_SUCCESS;
  }

  // If JSON is entered we do map matching
  MapMatcherFactory map_matcher_factory(pt);
  std::shared_ptr<valhalla::meili::MapMatcher> matcher(
      map_matcher_factory.Create(costing, request.options()));

  uint32_t i = 0;
  for (const auto& path : paths) {
    std::cout << "==========================================================================\n";
    std::cout << "                                 PATH " << i << std::endl;
    std::cout << "==========================================================================\n\n";
    std::vector<Measurement> measurements;
    measurements.reserve(path.size());
    for (const auto& location : path) {
      measurements.emplace_back(Measurement{{location.latlng_.lng(), location.latlng_.lat()},
                                            matcher->config().get<float>("gps_accuracy") + 10,
                                            matcher->config().get<float>("search_radius") + 10});
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
