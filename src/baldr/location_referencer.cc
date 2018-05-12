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

// Candidate edges for the start and end. Includes the path edge plus
// distance from segment start/end


uint16_t bearing(const std::vector<midgard::PointLL> &shape) {
  // OpenLR says to use 20m along the edge, but we could use the
  // GetOffsetForHeading function, which adapts it to the road class.
  float heading = shape.size() >= 2 ?
      midgard::PointLL::HeadingAlongPolyline(shape, 20) : 0.0f;
  assert(heading >= 0.0);
  assert(heading < 360.0);
  return uint16_t(std::round(heading));
}

uint16_t bearing(const baldr::GraphTile *tile, baldr::GraphId edge_id, float dist) {
  const auto *edge = tile->directededge(edge_id);
  std::vector<midgard::PointLL> shape = tile->edgeinfo(edge->edgeinfo_offset()).shape();
  if (!edge->forward()) {
    std::reverse(shape.begin(), shape.end());
  }
  if (dist > 0.0) {
    // Trim the shape and get bearing at the location along the edge
    trim_front(shape, uint32_t(dist * edge->length()));
  }
  return bearing(shape);
}

// given two bearings in degrees, return the unsigned angle between them.
int bear_diff(int bear1, int bear2) {
  int bear_diff = std::abs(bear1 - bear2);
  if (bear_diff > 180) {
    bear_diff = 360 - bear_diff;
  }
  if (bear_diff < 0) {
    bear_diff += 360;
  }
  return bear_diff;
}

// given two uint32_t, return the absolute difference between them, which will
// always fit into another uint32_t.
inline uint32_t abs_u32_diff(const uint32_t a, const uint32_t b) {
  return (a > b) ? (a - b) : (b - a);
}

// Attempt to provide a match score - how close is the match in total length,
// distance of the start and end of the path from the segment start and end.
uint32_t match_score(const float length_diff, const float origin_diff,
                     const float dest_diff) {
  return length_diff * length_diff + origin_diff * origin_diff + dest_diff * dest_diff;
}

// Edge association



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

class DistanceOnlyCost : public sif::DynamicCost {
public:
  DistanceOnlyCost(sif::TravelMode travel_mode);
  virtual ~DistanceOnlyCost();
  uint32_t access_mode() const;
  bool Allowed(const baldr::DirectedEdge* edge,
               const sif::EdgeLabel& pred,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid,
               const uint32_t current_time) const;
  bool AllowedReverse(const baldr::DirectedEdge* edge,
                      const sif::EdgeLabel& pred,
                      const baldr::DirectedEdge* opp_edge,
                      const baldr::GraphTile*& tile,
                      const baldr::GraphId& edgeid,
                      const uint32_t current_time) const;
  bool Allowed(const baldr::NodeInfo* node) const;
  sif::Cost EdgeCost(const baldr::DirectedEdge* edge) const;
  const sif::EdgeFilter GetEdgeFilter() const;
  const sif::NodeFilter GetNodeFilter() const;
  float AStarCostFactor() const;
};

DistanceOnlyCost::DistanceOnlyCost(sif::TravelMode travel_mode)
  : DynamicCost(boost::property_tree::ptree(), travel_mode) {
}

DistanceOnlyCost::~DistanceOnlyCost() {
}

uint32_t DistanceOnlyCost::access_mode() const {
  return baldr::kVehicularAccess;
}

bool DistanceOnlyCost::Allowed(const baldr::DirectedEdge* edge,
                               const sif::EdgeLabel& pred,
                               const baldr::GraphTile*&,
                               const baldr::GraphId&,
                               const uint32_t) const {
  // Do not allow U-turns/back-tracking
  return allow_edge_pred(edge) &&
        (pred.opp_local_idx() != edge->localedgeidx());
}

bool DistanceOnlyCost::AllowedReverse(const baldr::DirectedEdge* edge,
                                      const sif::EdgeLabel& pred,
                                      const baldr::DirectedEdge* opp_edge,
                                      const baldr::GraphTile*& tile,
                                      const baldr::GraphId& edgeid,
                                      const uint32_t) const {
  // Do not allow U-turns/back-tracking
  return allow_edge_pred(edge) &&
        (pred.opp_local_idx() != edge->localedgeidx());
}

bool DistanceOnlyCost::Allowed(const baldr::NodeInfo*) const {
  return true;
}

sif::Cost DistanceOnlyCost::EdgeCost(const baldr::DirectedEdge* edge) const {
  float edge_len(edge->length());
  return {edge_len, edge_len};
}

const sif::EdgeFilter DistanceOnlyCost::GetEdgeFilter() const {
  return [](const baldr::DirectedEdge *edge) -> float {
    return allow_edge_pred(edge) ? 1.0f : 0.0f;
  };
}

const sif::NodeFilter DistanceOnlyCost::GetNodeFilter() const {
  return [](const baldr::NodeInfo *) -> bool {
    return false;
  };
}

float DistanceOnlyCost::AStarCostFactor() const {
  return 1.0f;
}

/*
inline midgard::PointLL coord_for_lrp(const pbf::Segment::LocationReference &lrp) {
  return { static_cast<double>(lrp.coord().lng()) * 0.0000001,
           static_cast<double>(lrp.coord().lat()) * 0.0000001 };
}
*/

baldr::Location location_for_lrp(const valhalla::midgard::PointLL &ll, int bearing, int tolerance) {
  baldr::Location location(ll);
  // TODO: re-enable this once the value is correct 
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

struct last_tile_cache {
  last_tile_cache(baldr::GraphReader &reader)
      : m_reader(reader),
        m_last_tile(nullptr) {
  }

  inline const baldr::GraphTile *get(baldr::GraphId id) {
    if (id.Tile_Base() != m_last_id) {
      m_last_id = id.Tile_Base();
      m_last_tile = m_reader.GetGraphTile(m_last_id);
    }
    return m_last_tile;
  }

private:
  baldr::GraphReader &m_reader;
  baldr::GraphId m_last_id;
  const baldr::GraphTile *m_last_tile;
};

// Find nodes on the specified level that are within a specified distance
// from the lat,lon location
std::vector<baldr::GraphId> find_nearby_nodes(baldr::GraphReader& reader,
                              const midgard::PointLL& pt,
                              const uint8_t level) {
  // Create a bounding box and find nodes within the bounding box
  float meters_per_lng = midgard::DistanceApproximator::MetersPerLngDegree(pt.lat());
  float delta_lng = kNodeDistanceTolerance / meters_per_lng;
  float delta_lat = kNodeDistanceTolerance / midgard::kMetersPerDegreeLat;
  midgard::AABB2<midgard::PointLL> bbox({pt.lng() - delta_lng, pt.lat() - delta_lat},
                              {pt.lng() + delta_lng, pt.lat() + delta_lat});
  auto nodes = loki::nodes_in_bbox(bbox, reader);

  // Remove nodes that are not on the specified level
  std::vector<baldr::GraphId> level_nodes;
  for (const auto node : nodes) {
    if (node.level() == level) {
      level_nodes.emplace_back(node);
    }
  }
  return level_nodes;
}

// Add edges from each node
std::vector<CandidateEdge> GetEdgesFromNodes(baldr::GraphReader& reader,
                                    const std::vector<baldr::GraphId>& nodes,
                                    const PointLL& seg_coord,
                                    const bool origin) {
  last_tile_cache cache(reader);
  uint32_t count, edge_index;
  PointLL dmy;
  std::vector<CandidateEdge> edges;
  for (auto node : nodes) {
    auto* tile = cache.get(node);
    const NodeInfo* nodeinfo = tile->node(node);
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, ++edgeid) {
      // Skip non-regular edges - must be a road or ramp
      if (directededge->trans_up() || directededge->trans_down() ||
          directededge->is_shortcut() || directededge->roundabout() ||
         (directededge->use() != baldr::Use::kRoad && directededge->use() != baldr::Use::kRamp) ||
          directededge->internal()) {
        continue;
      }

      // If origin - add outbound edges that have vehicular access
      if (origin && (directededge->forwardaccess() & baldr::kVehicularAccess) != 0) {
        baldr::PathLocation::PathEdge edge(edgeid, 0.0f, dmy, 1.0f);
        edges.emplace_back(edge, seg_coord.Distance(nodeinfo->latlng()));
      }

      // If destination, add incoming, opposing edge if it has vehicular access
      if (!origin && (directededge->reverseaccess() & baldr::kVehicularAccess) != 0) {
        GraphId opp_edge_id = reader.GetOpposingEdgeId(edgeid);
        baldr::PathLocation::PathEdge edge(opp_edge_id, 1.0f, dmy, 1.0f);
        edges.emplace_back(edge, seg_coord.Distance(nodeinfo->latlng()));
      }
    }
  }
  return edges;
}

// Walk from the specified edge to the next one where there is only one choice.
// If there are more choices then just return invalid to signify stopping.
// This is trying to mimic OSMLR generation logic.
baldr::GraphId next_edge(const GraphId& edge_id, baldr::GraphReader& reader, const baldr::GraphTile*& tile,
                      uint32_t& edge_length) {
  if (tile->id() != edge_id.Tile_Base())
    tile = reader.GetGraphTile(edge_id);
  const auto* edge = tile->directededge(edge_id);

  // Get the end node (need a new tile if the edge leaves the current tile)
  GraphId end_node = edge->endnode();
  if (edge->leaves_tile())
    tile = reader.GetGraphTile(end_node);
  const auto* node = tile->node(end_node);

  // Get child edges of this node
  baldr::GraphId next;
  const auto* child_edge = tile->directededge(node->edge_index());
  for(int i = 0; i < node->edge_count(); ++i, child_edge++) {
    // Skip U-turns and edges that are not allowed
    if (i != edge->opp_index() && allow_edge_pred(child_edge)) {
      // Return invalid GraphId if more than 1 candidate edge exists
      if (next.Is_Valid())
        return {};
      else {
        next = {end_node.tileid(), end_node.level(), node->edge_index() + i};
        edge_length = child_edge->length();
      }
    }
  }
  return next;
}

// Edge association constructor
edge_association::edge_association(baldr::GraphReader &graphreader)
  : m_reader{graphreader},
    m_travel_mode(sif::TravelMode::kDrive),
    m_path_algo(new thor::AStarPathAlgorithm()),
    m_costing(new DistanceOnlyCost(m_travel_mode)) {
}

// Get a list of candidate edges for the location.
std::vector<CandidateEdge> edge_association::candidate_edges(bool origin,
                      const valhalla::midgard::PointLL &ll,
                      const double bearing,
                      const uint8_t level) {
  std::vector<CandidateEdge> edges;
  auto nodes = find_nearby_nodes(m_reader, ll, level);
  edges = GetEdgesFromNodes(m_reader, nodes, ll, origin);
  return edges;
}

std::vector<EdgeMatch> edge_association::match_edges(const valhalla::midgard::OpenLR::TwoPointLinearReference& locRef)
{
  // Try to match edges by walking a path from the first LRP to the last LRP
  // in the OSMLR segment. This uses a strategy similar to how OSMLR segments
  // are created
  std::vector<EdgeMatch> edges;
  const uint32_t expected_length = locRef.getLength();

  // check all the interim points of the location reference
  // TODO: figure out which level to pass in here
  auto origin = loki_search_single(baldr::Location(locRef.getFirstCoordinate()), m_reader, 0);
  auto dest = loki_search_single(
                  location_for_lrp(
                      locRef.getLastCoordinate(),
                      locRef.getLastBearing(),
                      kBearingTolerance),
                  m_reader,
                  0); // TODO figure out level
  if (dest.path_edges_size() == 0) {
    LOG_DEBUG("Unable to find edge near point " + std::to_string(locRef.getLastCoordinate()) +
              ". Segment cannot be matched, discarding.");
    return std::vector<EdgeMatch>();
  }

  // TODO - reject edges if bearing from origin is outside of tolerance?
  // TODO - do we need to use Form of Way in the Allowed costing method?

  // make sure there's no state left over from previous paths
  m_path_algo->Clear();
  m_path_algo->set_max_label_count(100);
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
    return std::vector<EdgeMatch>();
  }

  // Add edges to the matched path.
  // TODO - remove duplicate instances of the edge ID in the path info.
  // TODO: match partial % along edges
  for (const auto &info : path) {
    const GraphTile* tile = m_reader.GetGraphTile(info.edgeid);
    const DirectedEdge* edge = tile->directededge(info.edgeid);
    edges.emplace_back(EdgeMatch{info.edgeid, edge->length(), 0.0f, 1.0f});  // TODO - set first and last edge full_edge flag
  }
  return edges;
}

} // anonymous namespace

LocationReferencer::LocationReferencer(baldr::GraphReader& graphreader) : association(graphreader) {}

std::vector<EdgeMatch> LocationReferencer::match(const midgard::OpenLR::TwoPointLinearReference &locref)
{
  return association.match_edges(locref);
}

} // namespace baldr
} // namespace valhalla
