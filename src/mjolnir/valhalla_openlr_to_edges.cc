#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <thread>
#include <unordered_set>

#include <range/v3/all.hpp>

#include "baldr/graphconstants.h"
#include "baldr/openlr.h"
#include "baldr/rapidjson_utils.h"

#include "midgard/logging.h"
#include "sif/costfactory.h"

#include "loki/search.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/range/algorithm_ext.hpp>

#include "loki/worker.h"
#include "odin/worker.h"
#include "thor/worker.h"
#include "tyr/actor.h"
#include "midgard/polyline2.h"
#include "midgard/util.h"

const float LENGTH_TOLERANCE_M = 50;

std::unique_ptr<valhalla::thor::PathAlgorithm> get_path_algorithm(valhalla::baldr::GraphReader& reader,
		                                          const valhalla::Location& origin,
                                                  const valhalla::Location& dest) {

    for (auto& edge1 : origin.correlation().edges()) {
        for (auto& edge2 : dest.correlation().edges()) {
            bool same_graph_id = edge1.graph_id() == edge2.graph_id();
            bool are_connected = reader.AreEdgesConnected(valhalla::baldr::GraphId(edge1.graph_id()), valhalla::baldr::GraphId(edge2.graph_id()));
            if (same_graph_id || are_connected) {
                return std::make_unique<valhalla::thor::TimeDepForward>();
            }
        }
    }
    return std::make_unique<valhalla::thor::BidirectionalAStar>();
}

int main(int argc, char** argv) {

  namespace baldr = valhalla::baldr;
  namespace loki = valhalla::loki;
  namespace migard = valhalla::midgard;
  namespace sif = valhalla::sif;
  namespace thor = valhalla::thor;

  if (argc != 3) {
    LOG_ERROR("Usage: " + std::string(argv[0]) + " config/file.json openlr_binary");
    return 1;
  }

  // config file
  std::string config_file(argv[1]);
  boost::property_tree::ptree config;
  rapidjson::read_json(config_file, config);

  auto reader = std::make_shared<baldr::GraphReader>(config.get_child("mjolnir"));

  valhalla::Options default_options;
  sif::ParseCosting({}, "", default_options);
  default_options.set_costing_type(valhalla::Costing::auto_);
  default_options.set_search_radius(20);

  // Construct costing
  sif::CostFactory factory;
  sif::TravelMode mode;
  auto mode_costing = factory.CreateModeCosting(default_options, mode);

  auto lr = baldr::OpenLR::OpenLr(argv[2], true);

  auto lrp_to_location = [](const auto& lrp) -> baldr::Location {
    auto location = baldr::Location({lrp.longitude, lrp.latitude});
    location.node_snap_tolerance_ = 20;
    location.heading_ = lrp.bearing;
    location.heading_tolerance_ = 34;
    location.search_cutoff_ = 20;
    location.radius_ = 20;
    return location;
  };

  std::vector<baldr::Location> locations =
      lr.lrps | ranges::view::transform(lrp_to_location) | ranges::to<std::vector>();

  {
	  auto& last_location = locations.back();
	  //flip the bearing of the last lrp by 180
	  last_location.heading_ = *last_location.heading_ + 180.f;
	  last_location.heading_ = int(*last_location.heading_) % 360;
  }

  auto projections = loki::Search(locations, *reader, mode_costing[static_cast<size_t>(mode)]);

  std::cout << "projections size: " << projections.size() << std::endl;

  const auto main_roadclass =
      std::unordered_set<baldr::RoadClass>{baldr::RoadClass::kMotorway, baldr::RoadClass::kTrunk,
                                           baldr::RoadClass::kPrimary};
  const auto secondary_roadclass = std::unordered_set<baldr::RoadClass>{baldr::RoadClass::kSecondary};
  const auto tertiary_roadclass = std::unordered_set<baldr::RoadClass>{baldr::RoadClass::kTertiary};
  const auto other_roadclass = std::unordered_set<baldr::RoadClass>{baldr::RoadClass::kUnclassified,
                                                                    baldr::RoadClass::kResidential,
                                                                    baldr::RoadClass::kServiceOther};

  auto is_one_way = [](const baldr::DirectedEdge& edge) {
    auto fward = edge.forwardaccess() & baldr::kAutoAccess;
    auto bward = edge.reverseaccess() & baldr::kAutoAccess;
    return (fward != bward);
  };
  auto is_motor_way = [](const baldr::DirectedEdge& edge) {
    return edge.classification() == baldr::RoadClass::kMotorway;
  };

  auto frc_mismatches_roadclass = [&](auto frc, baldr::RoadClass rc) {
    if (frc == 0) {
      return main_roadclass.find(rc) == main_roadclass.end();
    }

    if (frc == 1) {
      return secondary_roadclass.find(rc) == secondary_roadclass.end();
    }

    if (frc == 2) {
      return tertiary_roadclass.find(rc) == tertiary_roadclass.end();
    }

    return other_roadclass.find(rc) == other_roadclass.end();
  };


  for (auto const& [idx, lrp] : lr.lrps | ranges::views::enumerate) {

    //const auto& idx = lrp_idx.first;
    //const auto& lrp = lrp_idx.second;

    std::cout << lrp.longitude << std::endl;
    std::cout << lrp.latitude << std::endl;

    // filter out the bad projection that are not in accordance with LRP
    auto rank = [&](const baldr::PathLocation::PathEdge& edge) -> int {
      // frc mismatch + (motorway mismatch || oneway mismatch || slip road mismatch)
      // FRC 0 – Main road
      // FRC 1 – First class road ...
      const auto* directed_edge = reader->directededge(edge.id);
      auto rank = frc_mismatches_roadclass(lrp.frc, directed_edge->classification()) +
                      !is_one_way(*directed_edge) ||
                  !is_motor_way(*directed_edge);
      return rank;
    };

    if (auto path_location = projections.find(locations[idx]); path_location == projections.end()) {
      std::cout << "ERROR" << std::endl;
      continue;
    } else {
      boost::range::remove_erase_if(path_location->second.edges,
                                    [rank](const auto& edge) { return rank(edge) > 1; });

      // process start lrp and end lrp
      if (idx == 0 && path_location->second.edges.size() > 1) {

    	auto at_end = [](const auto& edge) { return edge.percent_along == 1; };

    	auto count = ranges::count_if(path_location->second.edges, at_end);

    	if (count != path_location->second.edges.size()) {
        	// remove the edge if the location is projected at the end of the edge
            boost::range::remove_erase_if(path_location->second.edges, at_end);
    	}

      }
      if (idx == (lr.lrps.size() - 1) && path_location->second.edges.size() > 1) {


      	auto at_start= [](const auto& edge) { return edge.percent_along == 0; };

      	auto count = ranges::count_if(path_location->second.edges, at_start);

      	if (count != path_location->second.edges.size()) {
            // remove the edge if the location is projected at the beginning of the edge
            boost::range::remove_erase_if(path_location->second.edges, at_start);
      	}
      }

      std::sort(path_location->second.edges.begin(), path_location->second.edges.end(),
                [rank](const auto& lhs, const auto& rhs) { return rank(lhs) < rank(rhs); });

      std::cout << "path edges size: " << path_location->second.edges.size() << std::endl;

    }
  }


  auto first_lrp = lr.lrps.front();
  auto last_lrp = lr.lrps.back();

  auto first_location = locations.front();
  auto last_location = locations.back();

  auto first_edge_id = projections.find(first_location)->second.edges[0].id;

  std::cout << "first_edge_id: " << first_edge_id << std::endl;

  auto trivial_route =
      std::all_of(locations.cbegin(), locations.cend(), [&](const auto& location) {
	  	  auto edge_id =  projections.find(location)->second.edges[0].id;
	  	  std::cout << "edge_id: " << edge_id << std::endl;
	  	  return  edge_id  == first_edge_id;
      });

  if (trivial_route) {
    std::cout << "Lucky, a trivial route has been found" << std::endl;
    auto length =
        ranges::accumulate(locations | ranges::views::enumerate |
                               ranges::views::transform([reader, &projections, &locations](const auto& idx_location) -> uint32_t {
    							 const auto& [idx, location] = idx_location;
    							 const auto& path_location = projections.find(location)->second;
                                 if (path_location.edges.empty())
                                   return 0;
                                 const auto* direct_edge = reader->directededge(path_location.edges[0].id);

                                 if (idx == 0) {
                                     return direct_edge->length() * (1 - path_location.edges[0].percent_along);
                                 }

                                 if (idx == (locations.size() - 1)) {
                                     return direct_edge->length() * (path_location.edges[0].percent_along);
                                 }
                                 return direct_edge->length();
                               }),
                           0.);
    if ((lr.getLength() - LENGTH_TOLERANCE_M) < length &&
        length < (lr.getLength() + LENGTH_TOLERANCE_M)) {

    	std::cout << "length: " <<  length << std::endl;
    	std::cout << "lr.getLength: " <<  lr.getLength() << std::endl;
    	std::cout << first_edge_id << std::endl;
      return 0;
    }
  }
  // copy
  baldr::PathLocation start_path_location = projections.find(first_location)->second;
  baldr::PathLocation end_path_location = projections.find(last_location)->second;

  valhalla::Location origin;
  valhalla::Location dest;

  baldr::PathLocation::toPBF(start_path_location, &origin, *reader);
  baldr::PathLocation::toPBF(end_path_location, &dest, *reader);

  auto algo = get_path_algorithm(*reader, origin, dest);

  auto path_list = algo->GetBestPath(origin, dest, *reader, mode_costing, mode);

  valhalla::Api api;
  auto* trip_leg = api.mutable_trip()->mutable_routes()->Add()->mutable_legs()->Add();
  baldr::AttributesController controller;
  thor::TripLegBuilder::Build(default_options, controller, *reader, mode_costing, path_list[0].begin(), path_list[0].end(), origin, dest, *trip_leg, {"route"}, nullptr);

  double length = path_list[0].back().path_distance;
  std::cout << "length: " << length << std::endl;

  std::vector<std::string> edge_ids;
  for (const auto& p : path_list){
	  for (const auto info: p){
		  edge_ids.push_back(std::to_string(info.edgeid));

	  }
  }
  std::cout << "path_list size: " << path_list.size() << std::endl;

  std::cout << "lr.getLength: " << lr.getLength() << std::endl;

  if ((lr.getLength() - LENGTH_TOLERANCE_M) < length &&
		  length < (lr.getLength() + LENGTH_TOLERANCE_M)) {
	  auto joined = boost::algorithm::join(edge_ids, ", ");
	  std::cout << "result: " << joined << std::endl;
	  return 0;
  }

  return 0;
}
