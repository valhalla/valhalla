#include <cstdint>
#include <cmath>

#include "midgard/logging.h"
#include "baldr/graphreader.h"
#include "baldr/merge.h"
#include "loki/search.h"
#include "loki/node_search.h"
#include "thor/pathalgorithm.h"
#include "thor/astar.h"
#include "mjolnir/graphtilebuilder.h"
#include "midgard/openlr.h"

#include "baldr/location_referencer.h"

#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/iterator/reverse_iterator.hpp>

#include <deque>
#include <algorithm>
#include <thread>
#include <mutex>
#include <future>

#include "config.h"


namespace valhalla {
namespace baldr {


namespace {

// given two uint32_t, return the absolute difference between them, which will
// always fit into another uint32_t.
inline uint32_t abs_u32_diff(const uint32_t a, const uint32_t b) {
  return (a > b) ? (a - b) : (b - a);
}

// Use this method to determine whether an edge should be allowed along the
// merged path. This method should match the predicate used to create OSMLR
// segments.
bool allow_edge_pred(const baldr::DirectedEdge *edge) {
  return (!edge->trans_up() && !edge->trans_down() && !edge->is_shortcut() &&
           edge->classification() != baldr::RoadClass::kServiceOther &&
          (edge->use() == baldr::Use::kRoad || edge->use() == baldr::Use::kRamp) &&
          !edge->roundabout() && !edge->internal() &&
          (edge->forwardaccess() & baldr::kVehicularAccess) != 0);
}

enum class FormOfWay {
  kUndefined = 0,
  kMotorway = 1,
  kMultipleCarriageway = 2,
  kSingleCarriageway = 3,
  kRoundabout = 4,
  kTrafficSquare = 5,
  kSlipRoad = 6,
  kOther = 7
};

FormOfWay form_of_way(const baldr::DirectedEdge *e) {
  // Check if oneway. Assumes forward access is allowed. Edge is oneway if
  // no reverse vehicular access is allowed
  bool oneway = (e->reverseaccess() & baldr::kVehicularAccess) == 0;
  auto rclass = e->classification();

  // if it's a link (ramp or turn channel) return slip road
  if (e->link()) {
    return FormOfWay::kSlipRoad;
  }
  // if it's a roundabout, return that
  else if (e->roundabout()) {
    return FormOfWay::kRoundabout;
  }
  // if it's a motorway and it's one-way, then it's likely to be grade separated
  else if (rclass == baldr::RoadClass::kMotorway && oneway) {
    return FormOfWay::kMotorway;
  }
  // if it's a major road, and it's one-way then it might be a multiple
  // carriageway road.
  else if (rclass <= baldr::RoadClass::kTertiary && oneway) {
    return FormOfWay::kMultipleCarriageway;
  }
  // not one-way, so perhaps it's a single carriageway
  else if (rclass <= baldr::RoadClass::kTertiary) {
    return FormOfWay::kSingleCarriageway;
  }
  // everything else
  else {
    return FormOfWay::kOther;
  }
}

/*************************
 * Distance-based costing
 *************************/
class DistanceOnlyCost : public sif::DynamicCost {
public:
  DistanceOnlyCost(sif::TravelMode travel_mode)
    : DynamicCost(boost::property_tree::ptree(), travel_mode) {
  }

  virtual ~DistanceOnlyCost() { }
  uint32_t access_mode() const {
      return  baldr::kVehicularAccess;
  }
  bool Allowed(const baldr::DirectedEdge* edge,
               const sif::EdgeLabel& pred,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid,
               const uint32_t current_time) const 
  {
      return allow_edge_pred(edge) &&
          (pred.opp_local_idx() != edge->localedgeidx());
  };

  bool AllowedReverse(const baldr::DirectedEdge* edge,
                      const sif::EdgeLabel& pred,
                      const baldr::DirectedEdge* opp_edge,
                      const baldr::GraphTile*& tile,
                      const baldr::GraphId& edgeid,
                      const uint32_t current_time) const
  {
      // Do not allow U-turns/back-tracking
      return allow_edge_pred(edge) &&
            (pred.opp_local_idx() != edge->localedgeidx());
  }

  bool Allowed(const baldr::NodeInfo* node) const 
  {
      return true;
  }

  sif::Cost EdgeCost(const baldr::DirectedEdge* edge) const {
      float edge_len(edge->length());
      return {edge_len, edge_len};
  }

  const sif::EdgeFilter GetEdgeFilter() const {
      return [](const baldr::DirectedEdge *edge) -> float {
          return allow_edge_pred(edge) ? 1.0f : 0.0f;
      };
  }

  const sif::NodeFilter GetNodeFilter() const {
      return [](const baldr::NodeInfo *) -> bool {
          return false;
      };
  }
  float AStarCostFactor() const {
      return 1.0f;
  }
};

baldr::Location location_for_lrp(const valhalla::midgard::PointLL &ll, int bearing, int tolerance) {
  baldr::Location location(ll);
  location.heading_ = bearing;
  location.heading_tolerance_ = tolerance;
  return location;
}

odin::Location loki_search_single(const baldr::Location &loc, baldr::GraphReader &reader, uint8_t level) {
  //we dont want non real edges but also we want the edges to be on the right level
  //also right now only driveable edges please
  auto edge_filter = [level](const DirectedEdge* edge) -> float {
    return (edge->endnode().level() == level && allow_edge_pred(edge)) ? 1.0f : 0.0f;
  };

  //we only have one location so we only get one result
  std::vector<baldr::Location> locs{loc};
  locs.back().radius_ = kEdgeDistanceTolerance;
  baldr::PathLocation path_loc(loc);
  auto results = loki::Search(locs, reader, loki::PassThroughEdgeFilter, loki::PassThroughNodeFilter);
  if(results.size())
    path_loc = std::move(results.begin()->second);

  odin::Location l;
  baldr::PathLocation::toPBF(path_loc, &l, reader);

  return l;
}

} // anonymous namespace

LocationReferencer::LocationReferencer(baldr::GraphReader& graphreader)
  : m_reader{graphreader},
    m_travel_mode(sif::TravelMode::kDrive),
    m_path_algo(new thor::AStarPathAlgorithm()),
    m_costing(new DistanceOnlyCost(m_travel_mode)) 
{
}

std::vector<EdgeMatch> LocationReferencer::match(const midgard::OpenLR::TwoPointLinearReference &locRef)
{
  std::vector<EdgeMatch> edges;
  const uint32_t expected_length = locRef.getLength();

  // check all the interim points of the location reference
  // TODO: figure out which level to pass in here
  // TODO: origin search should consider outgoing bearing, dest search should consider incoming bearing
  // TODO - reject edges if bearing from origin is outside of tolerance?
  // TODO - do we need to use Form of Way in the Allowed costing method?
  auto origin = loki_search_single(baldr::Location(locRef.getFirstCoordinate()), m_reader, 0);
  if (origin.path_edges_size() == 0) {
    LOG_DEBUG("Unable to find edge near point " + std::to_string(locRef.getFirstCoordinate()) +
              ". Segment cannot be matched, discarding.");
    return std::vector<EdgeMatch>();
  }
  auto dest = loki_search_single(
                  location_for_lrp(
                      locRef.getLastCoordinate(),
                      locRef.getLastBearing(),
                      kSegmentSize / 2),
                  m_reader,
                  0); // TODO figure out level
  if (dest.path_edges_size() == 0) {
    LOG_DEBUG("Unable to find edge near point " + std::to_string(locRef.getLastCoordinate()) +
              ". Segment cannot be matched, discarding.");
    return std::vector<EdgeMatch>();
  }


  // make sure there's no state left over from previous paths
  m_path_algo->Clear();
  m_path_algo->set_max_label_count(100);

  // TODO: Should probably do a k-shortest-paths here.  If the shortest path is way
  // less than the expected path, how do we find the expected path?
  auto path = m_path_algo->GetBestPath(origin, dest, m_reader, &m_costing, m_travel_mode);
  if (path.empty()) {
    // what to do if there's no path?
    LOG_DEBUG("No route to destination " + std::to_string(locRef.getLastCoordinate()) + " from origin point " +
              std::to_string(coord) + ". Segment cannot be matched, discarding.");
    return std::vector<EdgeMatch>();
  }

  // Throw out if dist mismatch. The costing method stores distance in both
  // cost and elapsed time - so elapsed time of the last edge is total
  // path distance.
  if (abs_u32_diff(path.back().elapsed_time, expected_length) > kLengthToleranceMetres) {
    LOG_DEBUG("Found path of " + std::to_string(path.back().elapsed_time) + " is > " + std::to_string(kLengthToleranceMetres) +
        " different than the expected length of " + expected_length + ". Segment cannot be matched, discarding.");
    return std::vector<EdgeMatch>();
  }

  // Add edges to the matched path.
  // TODO: Remove duplicate instances of the edge ID in the path info.
  // TODO: Calculate proper % offsets at start/end edges according to origin/destination position
  for (const auto &info : path) {
    const GraphTile* tile = m_reader.GetGraphTile(info.edgeid);
    const DirectedEdge* edge = tile->directededge(info.edgeid);
    edges.emplace_back(EdgeMatch{info.edgeid, edge->length(), 0.0f, 1.0f});  // TODO - set first and last edge full_edge flag
  }
  return edges;
}

} // namespace baldr
} // namespace valhalla
