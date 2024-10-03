#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <range/v3/all.hpp>
#include <unordered_set>

#include "baldr/graphconstants.h"
#include "baldr/openlr.h"
#include "baldr/rapidjson_utils.h"
#include "loki/search.h"
#include "midgard/logging.h"
#include "midgard/polyline2.h"
#include "sif/costfactory.h"
#include "thor/bidirectional_astar.h"
#include "thor/unidirectional_astar.h"

const float LENGTH_TOLERANCE_M = 50;

namespace baldr = valhalla::baldr;
namespace loki = valhalla::loki;
namespace migard = valhalla::midgard;
namespace sif = valhalla::sif;
namespace thor = valhalla::thor;

namespace {

std::unique_ptr<valhalla::thor::PathAlgorithm>
get_path_algorithm(valhalla::baldr::GraphReader& reader,
                   const valhalla::Location& origin,
                   const valhalla::Location& dest) {

  for (auto& edge1 : origin.correlation().edges()) {
    for (auto& edge2 : dest.correlation().edges()) {
      bool same_graph_id = edge1.graph_id() == edge2.graph_id();
      bool are_connected = reader.AreEdgesConnected(valhalla::baldr::GraphId(edge1.graph_id()),
                                                    valhalla::baldr::GraphId(edge2.graph_id()));
      if (same_graph_id || are_connected) {
        return std::make_unique<valhalla::thor::TimeDepForward>();
      }
    }
  }
  return std::make_unique<valhalla::thor::BidirectionalAStar>();
}

valhalla::baldr::Location
lrp_to_location(const valhalla::baldr::OpenLR::LocationReferencePoint& lrp) {
  auto location = valhalla::baldr::Location({lrp.longitude, lrp.latitude});
  location.node_snap_tolerance_ = 20;
  location.heading_ = lrp.bearing;
  location.heading_tolerance_ = 34;
  location.search_cutoff_ = 20;
  location.radius_ = 20;
  return location;
}

bool is_one_way(const valhalla::baldr::DirectedEdge& edge) {
  auto fward = edge.forwardaccess() & valhalla::baldr::kAutoAccess;
  auto bward = edge.reverseaccess() & valhalla::baldr::kAutoAccess;
  return (fward != bward);
};

bool is_motor_way(const baldr::DirectedEdge& edge) {
  return edge.classification() == baldr::RoadClass::kMotorway;
};

bool is_link(const baldr::DirectedEdge& edge) {
  return edge.link();
};

size_t frc_mismatches_roadclass(unsigned char frc, baldr::RoadClass rc) {

  const auto main_roadclass =
      std::unordered_set<baldr::RoadClass>{baldr::RoadClass::kMotorway, baldr::RoadClass::kTrunk,
                                           baldr::RoadClass::kPrimary};
  const auto secondary_roadclass = std::unordered_set<baldr::RoadClass>{baldr::RoadClass::kSecondary};
  const auto tertiary_roadclass = std::unordered_set<baldr::RoadClass>{baldr::RoadClass::kTertiary};
  const auto other_roadclass = std::unordered_set<baldr::RoadClass>{baldr::RoadClass::kUnclassified,
                                                                    baldr::RoadClass::kResidential,
                                                                    baldr::RoadClass::kServiceOther};
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

// filter out the bad projection that are not in accordance with LRP
int rank(valhalla::baldr::GraphReader& reader,
         const valhalla::baldr::OpenLR::LocationReferencePoint& lrp,
         const baldr::PathLocation::PathEdge& edge) {
  // frc mismatch + (motorway mismatch || oneway mismatch || slip road mismatch)
  // FRC 0 – Main road
  // FRC 1 – First class road ...
  const auto* directed_edge = reader.directededge(edge.id);
  auto rank = frc_mismatches_roadclass(lrp.frc, directed_edge->classification()) +
                  !is_one_way(*directed_edge) ||
              !is_motor_way(*directed_edge) || !is_link(*directed_edge);
  return rank;
};

void print_final_result(int start_rank,
                        int end_rank,
                        const baldr::OpenLR::OpenLr& lr,
                        int length,
                        const std::vector<std::string>& edge_ids) {
  LOG_INFO("Found!");
  std::ostringstream ss;
  ss << "start rank: " << start_rank << " end rank: " << end_rank;
  ss << " lr.getLength: " << lr.getLength();
  ss << " length: " << length;
  ss << " edge_ids are: " << boost::algorithm::join(edge_ids, ", ") << std::endl;
  LOG_INFO(ss.str());
}

std::vector<std::vector<thor::PathInfo>> compute_path(const baldr::PathLocation& start,
                                                      const baldr::PathLocation& end,
                                                      const sif::mode_costing_t& mode_costing,
                                                      sif::TravelMode mode,
                                                      baldr::GraphReader& reader) {

  valhalla::Location origin;
  valhalla::Location dest;

  baldr::PathLocation::toPBF(start, &origin, reader);
  baldr::PathLocation::toPBF(end, &dest, reader);

  auto algo = get_path_algorithm(reader, origin, dest);
  return algo->GetBestPath(origin, dest, reader, mode_costing, mode);
}

} // namespace

int main(int argc, char** argv) {

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
  default_options.set_costing_type(valhalla::Costing::auto_);
  sif::ParseCosting({}, "", default_options);

  // Construct costing
  sif::TravelMode mode;
  auto mode_costing = sif::CostFactory().CreateModeCosting(default_options, mode);

  auto lr = baldr::OpenLR::OpenLr(argv[2], true);

  std::vector<baldr::Location> locations =
      lr.lrps | ranges::views::transform(lrp_to_location) | ranges::to<std::vector>();

  {
    // flip the bearing of the last lrp by 180
    auto& last_location = locations.back();
    last_location.heading_ = *last_location.heading_ + 180.f;
    last_location.heading_ = int(*last_location.heading_) % 360;
  }

  auto projections = loki::Search(locations, *reader, mode_costing[static_cast<size_t>(mode)]);

  assert(lr.lrps.size() == projections.size());

  LOG_INFO("projections size: " + std::to_string(projections.size()));

  auto lrp_ranked_path_edges =
      std::vector<std::unordered_map<size_t, std::vector<baldr::PathLocation::PathEdge>>>{};

  for (auto const& [idx, lrp] : lr.lrps | ranges::views::enumerate) {

    LOG_INFO("lrp longitude latitude: " + std::to_string(lrp.longitude) + "," +
             std::to_string(lrp.latitude));

    if (auto path_location = projections.find(locations[idx]); path_location == projections.end()) {
      const auto& latlng = locations[idx].latlng_;
      LOG_ERROR("Impossible to projection location: " + std::to_string(latlng.first) + " " +
                std::to_string(latlng.second));
      exit(1);
    } else {
      boost::range::remove_erase_if(path_location->second.edges, [lrp, &reader](const auto& edge) {
        return rank(*reader, lrp, edge) > 1;
      });

      // process start lrp and end lrp
      if (idx == 0 && path_location->second.edges.size() > 1) {

        auto at_end = [](const auto& edge) { return edge.percent_along == 1; };

        auto count = ranges::count_if(path_location->second.edges, at_end);

        if (static_cast<size_t>(count) != path_location->second.edges.size()) {
          // remove the edge if the location is projected at the end of the edge
          boost::range::remove_erase_if(path_location->second.edges, at_end);
        }
      }
      if (idx == (lr.lrps.size() - 1) && path_location->second.edges.size() > 1) {

        auto at_start = [](const auto& edge) { return edge.percent_along == 0; };

        auto count = ranges::count_if(path_location->second.edges, at_start);

        if (count != path_location->second.edges.size()) {
          // remove the edge if the location is projected at the beginning of the edge
          boost::range::remove_erase_if(path_location->second.edges, at_start);
        }
      }

      std::unordered_map<size_t, std::vector<baldr::PathLocation::PathEdge>> ranked_path_edges;

      for (auto edge : path_location->second.edges) {

        auto r = rank(*reader, lrp, edge);
        LOG_INFO("path edge id: " + std::to_string(edge.id) + " rank: " + std::to_string(r));
        ranked_path_edges[r].push_back(edge);
      }
      lrp_ranked_path_edges.push_back(ranked_path_edges);
    }
  }

  assert(lr.lrps.size() == lrp_ranked_path_edges.size());

  auto possible_trivial_edge_id = baldr::GraphId{baldr::kInvalidGraphId};

  auto first_percentage = 0.;
  auto last_percentage = 0.;

  if (auto first_0_rank_edges = lrp_ranked_path_edges[0].find(0);
      first_0_rank_edges != lrp_ranked_path_edges[0].end()) {

    for (const auto& first_edge : first_0_rank_edges->second) {

      auto find_trivial_route =
          ranges::all_of(lrp_ranked_path_edges | ranges::views::enumerate,
                         [&](const auto& idx_ranked_path_edges) {
                           auto& [idx, ranked_path_edges] = idx_ranked_path_edges;

                           auto rank_0_edges = ranked_path_edges.find(0);
                           if (rank_0_edges == ranked_path_edges.end())
                             return false;
                           for (const auto& edge : rank_0_edges->second) {
                             if (first_edge.id == edge.id) {
                               if (idx == 0) {
                                 first_percentage = edge.percent_along;
                               } else if (idx == (lrp_ranked_path_edges.size() - 1)) {
                                 last_percentage = edge.percent_along;
                               }
                               return true;
                             }
                           }
                           return false;
                         });
      if (find_trivial_route) {
        possible_trivial_edge_id = first_edge.id;
        break;
      }
    }
  }

  if (possible_trivial_edge_id.Is_Valid()) {

    auto trivial_route_length = reader->directededge(possible_trivial_edge_id)->length() *
                                (last_percentage - first_percentage);

    if (std::abs(trivial_route_length - lr.getLength()) <= LENGTH_TOLERANCE_M) {

      print_final_result(0, 0, lr, trivial_route_length, {std::to_string(possible_trivial_edge_id)});
      std::cout << possible_trivial_edge_id << std::endl;
      exit(0);
    }
  }

  LOG_INFO("No trivial route has been found");

  const auto& start_ranked_path_edges = lrp_ranked_path_edges.front();
  const auto& end_ranked_path_edges = lrp_ranked_path_edges.back();

  std::tuple<std::size_t, std::size_t, float, std::vector<baldr::GraphId>> best_res;

  auto first_location = locations.front();
  auto last_location = locations.back();

  // start rank and end rank are cartesian product of (0, 1, 2) and (0, 1, 2)
  auto rank_length_penality = 10;
  const auto& start_edges_map = lrp_ranked_path_edges.front();
  const auto& end_edges_map = lrp_ranked_path_edges.back();

  for (const auto& [start_rank, end_rank] :
       ranges::cartesian_product_view(ranges::views::ints(0, 3), ranges::views::ints(0, 3))) {
    {
      std::ostringstream ss;
      ss << "start rank: " << start_rank << " end rank: " << end_rank;
      LOG_INFO(ss.str());
    }
    const auto start_edges = start_edges_map.find(static_cast<size_t>(start_rank));
    const auto end_edges = end_edges_map.find(static_cast<size_t>(end_rank));

    if (start_edges == start_edges_map.end() || end_edges == end_edges_map.end()) {
      continue;
    }

    // copy
    baldr::PathLocation start_path_location = projections.find(first_location)->second;
    baldr::PathLocation end_path_location = projections.find(last_location)->second;

    start_path_location.edges = start_edges->second;
    end_path_location.edges = end_edges->second;

    auto path_list =
        compute_path(start_path_location, end_path_location, mode_costing, mode, *reader);

    if (path_list.empty() || path_list[0].empty())
      continue;

    double length = path_list[0].back().path_distance;

    if (std::abs(length - lr.getLength()) <= LENGTH_TOLERANCE_M) {
      std::vector<std::string> edge_ids;
      for (const auto& p : path_list) {
        for (const auto info : p) {
          edge_ids.push_back(std::to_string(info.edgeid));
        }
      }
      print_final_result(start_rank, end_rank, lr, length, edge_ids);
      std::cout << boost::algorithm::join(edge_ids, ", ") << std::endl;
      return 0;
    }
  }

  LOG_INFO("Last try");
  // last try:
  baldr::PathLocation start_path_location = projections.find(first_location)->second;
  baldr::PathLocation end_path_location = projections.find(last_location)->second;

  auto path_list = compute_path(start_path_location, end_path_location, mode_costing, mode, *reader);

  if (path_list.empty() || path_list[0].empty())
    exit(1);

  double length = path_list[0].back().path_distance;
  LOG_INFO("length: " + std::to_string(length));
  LOG_INFO("lr.getLength: " + std::to_string(lr.getLength()));

  if (std::abs(length - lr.getLength()) <= LENGTH_TOLERANCE_M) {
    std::vector<std::string> edge_ids;
    for (const auto& p : path_list) {
      for (const auto info : p) {
        edge_ids.push_back(std::to_string(info.edgeid));
      }
    }
    print_final_result(-1, -1, lr, length, edge_ids);
    std::cout << boost::algorithm::join(edge_ids, ", ") << std::endl;
    return 0;
  }

  exit(1);
}
