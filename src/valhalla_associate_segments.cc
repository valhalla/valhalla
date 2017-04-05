#include "midgard/logging.h"
#include "baldr/graphfsreader.h"
#include "baldr/tilefshierarchy.h"
#include "baldr/merge.h"
#include "loki/search.h"
#include "loki/node_search.h"
#include "thor/pathalgorithm.h"
#include "thor/astar.h"
#include "mjolnir/graphtilebuilder.h"

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
#include "segment.pb.h"
#include "tile.pb.h"

namespace vm = valhalla::midgard;
namespace vb = valhalla::baldr;
namespace vl = valhalla::loki;
namespace vs = valhalla::sif;
namespace vt = valhalla::thor;
namespace vj = valhalla::mjolnir;
namespace pbf = opentraffic::osmlr;

namespace bal = boost::algorithm;
namespace bpo = boost::program_options;
namespace bpt = boost::property_tree;
namespace bfs = boost::filesystem;

namespace std {
std::string to_string(const vm::PointLL &p) {
  std::ostringstream out;
  out.precision(16);
  out << "PointLL(" << p.lat() << ", " << p.lng() << ")";
  return out.str();
}

std::string to_string(const vb::GraphId &i) {
  std::ostringstream out;
  out << "GraphId(" << i.tileid() << ", " << i.level() << ", " << i.id() << ")";
  return out.str();
}
} // namespace std

namespace {

// Distance tolerance (meters) for node searching
// TODO - need to increase this to allow some tolerance for data edits.
// However - this sometimes causes fallbacks to A* (not sure why yet!)
constexpr float kNodeDistanceTolerance = 1.0;

// 10 meter length matching tolerance
constexpr uint32_t kLengthTolerance = 10;

// Bearing tolerance in degrees
constexpr uint16_t kBearingTolerance = 10;

enum class MatchType : uint8_t {
  kWalk = 0,
  kShortestPath = 1
};

// Matched edge
struct EdgeMatch {
  GraphId edgeid;
  uint32_t length;
  float start_pct;
  float end_pct;
};

// Candidate edges for the start and end. Includes the path edge plus
// distance from segment start/end
struct CandidateEdge {
  vb::PathLocation::PathEdge edge;
  float distance;

  CandidateEdge()
    : edge({}, 0.0f, {}, 1.0f),
      distance(0.0f) {
  }

  CandidateEdge(const vb::PathLocation::PathEdge& e, const float d)
      : edge(e),
        distance(d) {
  }
};

uint16_t bearing(const std::vector<vm::PointLL> &shape) {
  // OpenLR says to use 20m along the edge, but we could use the
  // GetOffsetForHeading function, which adapts it to the road class.
  float heading = vm::PointLL::HeadingAlongPolyline(shape, 20);
  assert(heading >= 0.0);
  assert(heading < 360.0);
  return uint16_t(std::round(heading));
}

uint16_t bearing(const vb::GraphTile *tile, vb::GraphId edge_id, float dist) {
  const auto *edge = tile->directededge(edge_id);
  std::vector<vm::PointLL> shape = tile->edgeinfo(edge->edgeinfo_offset()).shape();
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
using leftovers_t = std::vector<std::pair<vb::GraphId, vb::TrafficChunk > >;
using chunk_t = std::unordered_map<vb::GraphId, std::vector<vb::TrafficChunk> >;
using chunks_t = std::vector<std::pair<vb::GraphId, std::vector<vb::TrafficChunk > > >;
struct edge_association {
  explicit edge_association(const bpt::ptree &pt);

  // Associate a tile of OSMLR to Valhalla edges
  void add_tile(const std::string& file_name);

  // Get the "leftovers" - these are edge-segment associations where the
  // edges are not in the same tile as the OSMLR segment.
  const leftovers_t& leftovers() const { return m_leftover_associations; }

  // Get the chunks. These are edges that associate to more than 1 OSMLR
  // segment.
  const chunk_t& chunks() const { return traffic_chunks; }

  uint32_t success_count() const { return success_count_; }
  uint32_t failure_count() const { return failure_count_; }
  uint32_t walk_count() const { return walk_count_; }
  uint32_t path_count() const { return path_count_; }

private:
  std::vector<CandidateEdge> candidate_edges(bool origin,
                        const pbf::Segment::LocationReference &lrp,
                        const uint8_t level);
  bool match_segment(vb::GraphId segment_id, const pbf::Segment &segment,
                     MatchType& match_type);
  std::vector<EdgeMatch> match_edges(const pbf::Segment &segment,
                    const vb::GraphId& segment_id,
                    const uint32_t segment_length, MatchType& match_type);
  std::vector<EdgeMatch> walk(uint8_t level, const uint32_t segment_length,
                    const pbf::Segment& segment);
  vm::PointLL lookup_end_coord(const vb::GraphId& edge_id);
  vm::PointLL lookup_start_coord(const vb::GraphId& edge_id);

  vb::GraphFsReader m_reader;
  vs::TravelMode m_travel_mode;
  std::shared_ptr<vt::PathAlgorithm> m_path_algo;
  std::shared_ptr<vs::DynamicCost> m_costing;
  std::shared_ptr<vj::GraphTileBuilder> m_tile_builder;
  const vb::GraphTile* m_tile;

  // Statistics
  uint32_t success_count_;
  uint32_t failure_count_;
  uint32_t walk_count_;
  uint32_t path_count_;

  // Chunks - saved for later
  chunk_t traffic_chunks;

  // simple associations saved for later
  leftovers_t m_leftover_associations;
};

// Use this method to determine whether an edge should be allowed along the
// merged path. This method should match the predicate used to create OSMLR
// segments.
bool allow_edge_pred(const vb::DirectedEdge *edge) {
  return (!edge->trans_up() && !edge->trans_down() && !edge->is_shortcut() &&
           edge->use() != vb::Use::kTransitConnection &&
           edge->use() != vb::Use::kTurnChannel && !edge->internal() &&
           edge->use() != vb::Use::kFerry && !edge->roundabout() &&
          (edge->forwardaccess() & vb::kVehicularAccess) != 0);
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

FormOfWay form_of_way(const vb::DirectedEdge *e) {
  // Check if oneway. Assumes forward access is allowed. Edge is oneway if
  // no reverse vehicular access is allowed
  bool oneway = (e->reverseaccess() & vb::kVehicularAccess) == 0;
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
  else if (rclass == vb::RoadClass::kMotorway && oneway) {
    return FormOfWay::kMotorway;
  }
  // if it's a major road, and it's one-way then it might be a multiple
  // carriageway road.
  else if (rclass <= vb::RoadClass::kTertiary && oneway) {
    return FormOfWay::kMultipleCarriageway;
  }
  // not one-way, so perhaps it's a single carriageway
  else if (rclass <= vb::RoadClass::kTertiary) {
    return FormOfWay::kSingleCarriageway;
  }
  // everything else
  else {
    return FormOfWay::kOther;
  }
}

class DistanceOnlyCost : public vs::DynamicCost {
public:
  DistanceOnlyCost(vs::TravelMode travel_mode);
  virtual ~DistanceOnlyCost();
  uint32_t access_mode() const;
  bool Allowed(const vb::DirectedEdge* edge,
               const vs::EdgeLabel& pred,
               const vb::GraphTile*& tile,
               const vb::GraphId& edgeid) const;
  bool AllowedReverse(const vb::DirectedEdge* edge,
                      const vs::EdgeLabel& pred,
                      const vb::DirectedEdge* opp_edge,
                      const vb::GraphTile*& tile,
                      const vb::GraphId& edgeid) const;
  bool Allowed(const vb::NodeInfo* node) const;
  vs::Cost EdgeCost(const vb::DirectedEdge* edge) const;
  const vs::EdgeFilter GetEdgeFilter() const;
  const vs::NodeFilter GetNodeFilter() const;
  float AStarCostFactor() const;
};

DistanceOnlyCost::DistanceOnlyCost(vs::TravelMode travel_mode)
  : DynamicCost(bpt::ptree(), travel_mode) {
}

DistanceOnlyCost::~DistanceOnlyCost() {
}

uint32_t DistanceOnlyCost::access_mode() const {
  return vb::kVehicularAccess;
}

bool DistanceOnlyCost::Allowed(const vb::DirectedEdge* edge,
                               const vs::EdgeLabel& pred,
                               const vb::GraphTile*&,
                               const vb::GraphId&) const {
  // Do not allow U-turns/back-tracking
  return allow_edge_pred(edge) &&
        (pred.opp_local_idx() != edge->localedgeidx());
}

bool DistanceOnlyCost::AllowedReverse(const vb::DirectedEdge* edge,
                                      const vs::EdgeLabel& pred,
                                      const vb::DirectedEdge* opp_edge,
                                      const vb::GraphTile*& tile,
                                      const vb::GraphId& edgeid) const {
  // Do not allow U-turns/back-tracking
  return allow_edge_pred(edge) &&
        (pred.opp_local_idx() != edge->localedgeidx());
}

bool DistanceOnlyCost::Allowed(const vb::NodeInfo*) const {
  return true;
}

vs::Cost DistanceOnlyCost::EdgeCost(const vb::DirectedEdge* edge) const {
  float edge_len(edge->length());
  return {edge_len, edge_len};
}

const vs::EdgeFilter DistanceOnlyCost::GetEdgeFilter() const {
  return [](const vb::DirectedEdge *edge) -> float {
    return allow_edge_pred(edge) ? 1.0f : 0.0f;
  };
}

const vs::NodeFilter DistanceOnlyCost::GetNodeFilter() const {
  return [](const vb::NodeInfo *) -> bool {
    return false;
  };
}

float DistanceOnlyCost::AStarCostFactor() const {
  return 1.0f;
}

inline vm::PointLL coord_for_lrp(const pbf::Segment::LocationReference &lrp) {
  return { static_cast<double>(lrp.coord().lng()) * 0.0000001,
           static_cast<double>(lrp.coord().lat()) * 0.0000001 };
}

vb::Location location_for_lrp(const pbf::Segment::LocationReference &lrp) {
  vb::Location location({ static_cast<double>(lrp.coord().lng()) * 0.0000001,
                          static_cast<double>(lrp.coord().lat()) * 0.0000001 });
  if(lrp.has_bear())
    location.heading_ = lrp.bear();
  return location;
}

vb::PathLocation loki_search_single(const vb::Location &loc, vb::GraphReader &reader, uint8_t level) {
  //we dont want non real edges but also we want the edges to be on the right level
  //also right now only driveable edges please
  auto edge_filter = [level](const DirectedEdge* edge) -> float {
    return (edge->endnode().level() == level && allow_edge_pred(edge)) ? 1.0f : 0.0f;
  };

  //we only have one location so we only get one result
  std::vector<vb::Location> locs{loc};
  vb::PathLocation path_loc(loc);
  auto results = vl::Search(locs, reader, edge_filter, vl::PassThroughNodeFilter);
  if(results.size())
    path_loc = std::move(results.begin()->second);

  return path_loc;
}

struct last_tile_cache {
  last_tile_cache(vb::GraphReader &reader)
      : m_reader(reader),
        m_last_tile(nullptr) {
  }

  inline const vb::GraphTile *get(vb::GraphId id) {
    if (id.Tile_Base() != m_last_id) {
      m_last_id = id.Tile_Base();
      m_last_tile = m_reader.GetGraphTile(m_last_id);
    }
    return m_last_tile;
  }

private:
  vb::GraphReader &m_reader;
  vb::GraphId m_last_id;
  const vb::GraphTile *m_last_tile;
};

// Find nodes within a specified distance of lat,lon location
std::vector<vb::GraphId> find_nearby_nodes(vb::GraphReader& reader,
                              const float dist, const vm::PointLL& pt) {
  // Create a bounding box
  float meters_per_lng = vm::DistanceApproximator::MetersPerLngDegree(pt.lat());
  float delta_lng = kNodeDistanceTolerance / meters_per_lng;
  float delta_lat = kNodeDistanceTolerance / vm::kMetersPerDegreeLat;
  vm::AABB2<vm::PointLL> bbox({pt.lng() - delta_lng, pt.lat() - delta_lat},
                              {pt.lng() + delta_lng, pt.lat() + delta_lat});
  auto nodes = vl::nodes_in_bbox(bbox, reader);

  // Remove nodes on the local level (TODO-make this configurable?)
  std::vector<vb::GraphId> non_local_nodes;
  for (const auto node : nodes) {
    if (node.level() < 2) {
      non_local_nodes.emplace_back(node);
    }
  }
  return non_local_nodes;
}

// Add edges from each node
std::vector<CandidateEdge> GetEdgesFromNodes(vb::GraphReader& reader,
                                    const std::vector<vb::GraphId>& nodes,
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
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, edgeid++) {
      // Skip non-regular edges
      if (directededge->trans_up() || directededge->trans_down() ||
          directededge->is_shortcut() || directededge->roundabout() ||
          directededge->use() == vb::Use::kTransitConnection ||
          directededge->use() == vb::Use::kTurnChannel ||
          directededge->internal() || directededge->use() == vb::Use::kFerry) {
        continue;
      }

      // If origin - add outbound edges that have vehicular access
      if (origin && (directededge->forwardaccess() & vb::kVehicularAccess) != 0) {
        vb::PathLocation::PathEdge edge(edgeid, 0.0f, dmy, 1.0f);
        edges.emplace_back(edge, seg_coord.Distance(nodeinfo->latlng()));
      }

      // If destination, add incoming, opposing edge if it has vehicular access
      if (!origin && (directededge->reverseaccess() & vb::kVehicularAccess) != 0) {
        GraphId opp_edge_id = reader.GetOpposingEdgeId(edgeid);
        vb::PathLocation::PathEdge edge(opp_edge_id, 1.0f, dmy, 1.0f);
        edges.emplace_back(edge, seg_coord.Distance(nodeinfo->latlng()));
      }
    }
  }
  return edges;
}

// Walk from the specified edge to the next one where there is only one choice.
// If there are more choices then just return invalid to signify stopping.
// This is trying to mimic OSMLR generation logic.
vb::GraphId next_edge(const GraphId& edge_id, vb::GraphReader& reader, const vb::GraphTile*& tile,
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
  vb::GraphId next;
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
edge_association::edge_association(const bpt::ptree &pt)
  : success_count_(0),
    failure_count_(0),
    walk_count_(0),
    path_count_(0),
    m_reader(pt.get_child("mjolnir")),
    m_travel_mode(vs::TravelMode::kDrive),
    m_path_algo(new vt::AStarPathAlgorithm()),
    m_costing(new DistanceOnlyCost(m_travel_mode)),
    m_tile(nullptr) {
}

// Get a list of candidate edges for the location.
std::vector<CandidateEdge> edge_association::candidate_edges(bool origin,
                      const pbf::Segment::LocationReference &lrp,
                      const uint8_t level) {
  auto ll = coord_for_lrp(lrp);
  std::vector<CandidateEdge> edges;
  if (lrp.at_node()) {
    // Find nearby nodes and get allowed edges
    auto nodes = find_nearby_nodes(m_reader, kNodeDistanceTolerance, ll);
    edges = GetEdgesFromNodes(m_reader, nodes, ll, origin);
  } else {
    // Use edge search with loki
    auto loc = loki_search_single(vb::Location(ll), m_reader, level);
    for (const auto& edge : loc.edges) {
      edges.emplace_back(edge, 0.0f);  // TODO??
    }
    if (origin) {
      // Remove inbound edges to an origin node
      std::remove_if(edges.begin(), edges.end(),
         [](const CandidateEdge& e){return e.edge.end_node();});
    } else {
      // remove outbound edges to a destination node
      std::remove_if(edges.begin(), edges.end(),
         [](const CandidateEdge& e){return e.edge.begin_node();});
    }
  }
  return edges;
}

// Walk a path between origin and destination edges.
std::vector<EdgeMatch> edge_association::walk(uint8_t level,
                            const uint32_t segment_length,
                            const pbf::Segment& segment) {
  // Get the candidate origin and destination edges
  size_t n = segment.lrps_size();
  auto origin_edges = candidate_edges(true, segment.lrps(0), level);
  auto destination_edges = candidate_edges(false, segment.lrps(n-1), level);

  // Create a map of candidate destination edges for faster lookup
  std::unordered_map<GraphId, CandidateEdge> dest_edges;
  for (const auto& dest_edge : destination_edges) {
    dest_edges[dest_edge.edge.id] = dest_edge;
  }

  // Walk edges until there is not a continuing edge (means there are 0 or
  // more than 1 continuing edge) or we have exceeded the length. Keep track
  // of the best matched path.
  uint32_t edge_length = 0;
  uint32_t best_score = std::numeric_limits<uint32_t>::max();
  uint32_t max_length = segment_length + kLengthTolerance;
  FormOfWay fow = FormOfWay(segment.lrps(0).start_fow());
  vb::RoadClass road_class = vb::RoadClass(segment.lrps(0).start_frc());
  std::vector<EdgeMatch> best_path;
  for (const auto& origin_edge : origin_edges) {
    // Check that this edge matches road class and form of way
    // TODO - what are implications for data updates - perhaps allow
    // differing values but use this in the "scoring"
    auto* tile = m_reader.GetGraphTile(origin_edge.edge.id);
    const auto* edge = tile->directededge(origin_edge.edge.id);
    if (road_class != edge->classification() ||  fow != form_of_way(edge)) {
      continue;
    }

    // Check the bearing for this edge and make sure within tolerance
    // TODO - short edges have bearing inaccuracy (what is too short?)
    if (segment_length > 5 && edge->length() > 5) {
      uint16_t walked_bearing = bearing(tile, origin_edge.edge.id, origin_edge.edge.dist);
      // Increase bearing tolerance if LRP is not at a node and edge length is long
      float tolerance = kBearingTolerance;
      if (!segment.lrps(0).at_node() && edge->length() > 1000) {
        tolerance = 45.0f;
      }
      if (bear_diff(walked_bearing, segment.lrps(0).bear()) > tolerance) {
        continue;
      }
    }

    // Walk a path from this origin edge.
    std::vector<EdgeMatch> edges;
    edges.emplace_back(EdgeMatch{origin_edge.edge.id, edge->length(), origin_edge.edge.dist, 1.0f});

    // Check if the origin edge matches a destination edge
    auto dest = dest_edges.find(origin_edge.edge.id);
    if (dest != dest_edges.end()) {
      // Check if this path is a better match
      uint32_t walked_length = edge->length() * (dest->second.edge.dist - origin_edge.edge.dist);
      uint32_t d = abs_u32_diff(walked_length, segment_length);
      if (d < kLengthTolerance) {
        uint32_t score = match_score(d, origin_edge.distance, dest->second.distance);
        if (score < best_score) {
          best_path = edges;
          best_path.back().end_pct = dest->second.edge.dist;
          best_score = score;
        }
      }
    }

    // Continue even if the origin edge matches a destination edge - this could
    // be to a different nearby node but it is not the optimal one.
    uint32_t walked_length = edge->length() * (1.0f - origin_edge.edge.dist);
    while (true) {
      // Get the next edge. Break if next edge is invalid.
      GraphId edgeid = next_edge(edges.back().edgeid, m_reader, tile, edge_length);
      if (!edgeid.Is_Valid()) {
        break;
      }

      // Check if this is a destination edge
      auto dest = dest_edges.find(edgeid);
      if (dest != dest_edges.end()) {
        // Check if this path is within tolerance and a better match
        uint32_t length = walked_length + (edge_length * dest->second.edge.dist);
        uint32_t d = abs_u32_diff(length, segment_length);
        if (d < kLengthTolerance) {
          uint32_t score = match_score(d, origin_edge.distance, dest->second.distance);
          if (score < best_score) {
            best_path = edges;
            best_path.emplace_back(EdgeMatch{edgeid, edge_length, 0.0f, dest->second.edge.dist});
            best_score = score;
          }
        }
      }

      // Add the full edge to the path - break if exceeds max length.
      walked_length += edge_length;
      if (walked_length > max_length) {
        break;
      }
      edges.emplace_back(EdgeMatch{edgeid, edge_length, 0.0f, 1.0f});
    }
  }
  // Edge walking did not find a candidate path
  return best_path;
}

// Match the OSMLR segment to Valhalla edges.
std::vector<EdgeMatch> edge_association::match_edges(const pbf::Segment& segment,
                            const vb::GraphId& segment_id,
                            const uint32_t segment_length, MatchType& match_type) {
  const size_t size = segment.lrps_size();
  assert(size >= 2);

  // Try to match edges by walking a path from the first LRP to the last LRP
  // in the OSMLR segment. This uses a strategy similar to how OSMLR segments
  // are created
  std::vector<EdgeMatch> edges = walk(segment_id.level(), segment_length, segment);
  if (edges.size()) {
    match_type = MatchType::kWalk;
    return edges;
  }

  // TODO - this should not happen if both are at nodes! Does not happen
  // with low node tolerance, but as node search tolerance is raised we get
  // fallback cases where we shouldn't
  if (segment.lrps(0).at_node() && segment.lrps(size-1).at_node()) {
    LOG_INFO("Fall back to A*: " + std::to_string(segment_id) + " value = " +
                  std::to_string(segment_id.value));
  }

  // Fall back to A* shortest path to form the path edges

  // check all the interim points of the location reference
  auto origin_coord = coord_for_lrp(segment.lrps(0));
  auto origin = loki_search_single(vb::Location(origin_coord), m_reader, segment_id.level());
  for (size_t i = 0; i < size - 1; ++i) {
    auto &lrp = segment.lrps(i);
    auto coord = coord_for_lrp(lrp);
    auto next_coord = coord_for_lrp(segment.lrps(i+1));

    vb::RoadClass road_class = vb::RoadClass(lrp.start_frc());

    auto dest = loki_search_single(location_for_lrp(segment.lrps(i+1)), m_reader, segment_id.level());
    if (dest.edges.size() == 0) {
      LOG_DEBUG("Unable to find edge near point " + std::to_string(next_coord) + ". Segment cannot be matched, discarding.");
      return std::vector<EdgeMatch>();
    }

    // make sure there's no state left over from previous paths
    m_path_algo->Clear();
    auto path = m_path_algo->GetBestPath(origin, dest, m_reader, &m_costing, m_travel_mode);

    if (path.empty()) {
      // what to do if there's no path?
      LOG_DEBUG("No route to destination " + std::to_string(next_coord) + " from origin point " + std::to_string(coord) + ". Segment cannot be matched, discarding.");
      return std::vector<EdgeMatch>();
    }

    {
      auto last_edge_id = path.back().edgeid;
      auto *tile = m_reader.GetGraphTile(last_edge_id);
      auto *edge = tile->directededge(last_edge_id);
      auto node_id = edge->endnode();
      auto *ntile = (last_edge_id.Tile_Base() == node_id.Tile_Base()) ? tile : m_reader.GetGraphTile(node_id);
      auto *node = ntile->node(node_id);
      auto dist = node->latlng().Distance(next_coord);
      if (dist > 10.0f) {
        LOG_DEBUG("Route to destination " + std::to_string(next_coord) + " from origin point " + std::to_string(coord) + " ends more than 10m away: " + std::to_string(node->latlng()) + ". Segment cannot be matched, discarding.");
        return std::vector<EdgeMatch>();
      }
    }

    int score = 0;
    uint32_t sum = 0;
    for (auto &p : path) {
      sum += p.elapsed_time;
    }
    score += std::abs(int(sum) - int(lrp.length())) / 10;

    auto edge_id = path.front().edgeid;
    auto *tile = m_reader.GetGraphTile(edge_id);
    auto *edge = tile->directededge(edge_id);

    if (!allow_edge_pred(edge)) {
      LOG_DEBUG("Edge " + std::to_string(edge_id) + " not accessible. Segment cannot be matched, discarding.");
      return std::vector<EdgeMatch>();
    }
    score += std::abs(int(road_class) - int(edge->classification()));

    bool found = false;
    for (auto &e : origin.edges) {
      if (e.id == edge_id) {
        found = true;
        score += int(e.projected.Distance(coord));

        int bear1 = bearing(tile, edge_id, e.dist);
        int bear2 = lrp.bear();
        score += bear_diff(bear1, bear2) / 10;

        break;
      }
    }
    if (!found) {
      LOG_DEBUG("Unable to find edge " + std::to_string(edge_id) + " at origin point " + std::to_string(origin.latlng_) + ". Segment cannot be matched, discarding.");
      return std::vector<EdgeMatch>();
    }

    // form of way isn't really a metric space...
    FormOfWay fow1 = form_of_way(edge);
    FormOfWay fow2 = FormOfWay(lrp.start_fow());
    score += (fow1 == fow2) ? 0 : 5;

    for (const auto &info : path) {
      const GraphTile* tile = m_reader.GetGraphTile(info.edgeid);
      const DirectedEdge* edge = tile->directededge(info.edgeid);
      edges.emplace_back(EdgeMatch{info.edgeid, edge->length(), 0.0f, 1.0f});  // TODO - set first and last edge full_edge flag
    }

    // use dest as next origin
    std::swap(origin, dest);
  }

  // remove duplicate instances of the edge ID in the path info. TODO
//   auto new_end = std::unique(edges.begin(), edges.end());
//  edges.erase(new_end, edges.end());
  match_type = MatchType::kShortestPath;
  return edges;
}

vm::PointLL edge_association::lookup_end_coord(const vb::GraphId& edge_id) {
  auto *tile = m_reader.GetGraphTile(edge_id);
  auto *edge = tile->directededge(edge_id);
  auto node_id = edge->endnode();
  auto *node_tile = tile;
  if (edge_id.Tile_Base() != node_id.Tile_Base()) {
    node_tile = m_reader.GetGraphTile(node_id);
  }
  auto *node = node_tile->node(node_id);
  return node->latlng();
}

vm::PointLL edge_association::lookup_start_coord(const vb::GraphId& edge_id) {
  auto *tile = m_reader.GetGraphTile(edge_id);
  auto *edge = tile->directededge(edge_id);
  auto opp_index = edge->opp_index();
  auto node_id = edge->endnode();
  auto *node_tile = tile;
  if (edge_id.Tile_Base() != node_id.Tile_Base()) {
    node_tile = m_reader.GetGraphTile(node_id);
  }
  auto *node = node_tile->node(node_id);
  return lookup_end_coord(node_id.Tile_Base() + uint64_t(node->edge_index() + opp_index));
}

bool edge_association::match_segment(vb::GraphId segment_id, const pbf::Segment &segment,
                                     MatchType& match_type) {
  // Get the total length of the OSMLR segment
  uint32_t segment_length = 0;
  for (const auto& lrp : segment.lrps()) {
    segment_length += lrp.length();
  }

  auto edges = match_edges(segment, segment_id, segment_length, match_type);
  if (edges.empty()) {
    size_t n = segment.lrps_size();
    if (segment.lrps(0).at_node() && segment.lrps(n-1).at_node()) {
      LOG_INFO("No match from nodes: " + std::to_string(segment_id) + " value = " +
                    std::to_string(segment_id.value));
    } else {
      LOG_INFO("No match along edge: " + std::to_string(segment_id) + " value = " +
              std::to_string(segment_id.value));
    }
    return false;
  }

  // Walk the matched edges. Any full edges get associated to the segment.
  // Partial edges get saved for later
  for (size_t i = 0; i < edges.size(); ++i) {
    // Form association for this edge. First edge "starts"
    // the traffic segment and the last edge ends the segment.
    const auto& edge = edges[i];
    vb::TrafficChunk assoc(segment_id, edge.start_pct, edge.end_pct, i == 0, i == (edges.size() - 1));
    // Full edge.
    if (edge.start_pct == 0.0f && edge.end_pct == 1.0f) {
      // Store the local segment and leave the non local for later
      if(m_tile_builder->id() == edge.edgeid.Tile_Base())
        m_tile_builder->AddTrafficSegment(edge.edgeid, assoc);
      else
        m_leftover_associations.emplace_back(std::make_pair(edge.edgeid, std::move(assoc)));
    }// Add the temporary chunk information for this edge to the map
    else
      traffic_chunks[edge.edgeid].emplace_back(std::move(assoc));
  }
  return true;
}

vb::GraphId parse_file_name(const std::string &file_name) {
  return vb::GraphTile::GetTileId(file_name);
}

void edge_association::add_tile(const std::string &file_name) {
  //read the osmlr tile
  pbf::Tile tile;
  {
    std::ifstream in(file_name);
    if (!tile.ParseFromIstream(&in)) {
      throw std::runtime_error("Unable to parse traffic segment file.");
    }
  }

  //get a tile builder ready for this tile
  auto base_id = parse_file_name(file_name);
  m_tile_builder.reset(new vj::GraphTileBuilder(m_reader.GetTileHierarchy(), base_id, false));
  m_tile_builder->InitializeTrafficSegments();
  m_tile = m_reader.GetGraphTile(base_id);

  //do the matching of the segments in this osmlr tile
  std::cout.precision(16);
  uint64_t entry_id = 0;
  MatchType match_type = MatchType::kWalk;
  for (auto &entry : tile.entries()) {
    if (!entry.has_marker()) {
      assert(entry.has_segment());
      auto &segment = entry.segment();
      if (match_segment(base_id + entry_id, segment, match_type)) {
        success_count_++;
        if (match_type == MatchType::kWalk) {
          walk_count_++;
        } else {
          path_count_++;
        }
      } else {
        failure_count_++;
      }
    }
    entry_id += 1;
  }

  //finish this tile
  m_reader.Clear();
  m_tile_builder->UpdateTrafficSegments();
}

// Add OSMLR segment associations to each tile in the list. Any "local"
// associations where the edge is within the same tile as the OSMLR segment
// are written to the tile.
void add_local_associations(const bpt::ptree &pt, std::deque<std::string>& osmlr_tiles, std::mutex& lock,
  std::promise<edge_association>& association) {

  //this holds the extra data before we serialize it to the extra section
  //of a tile.
  edge_association e(pt);

  while(true) {
    //get a file to work with
    std::string osmlr_filename;
    lock.lock();
    if(osmlr_tiles.size()) {
      osmlr_filename = std::move(osmlr_tiles.front());
      osmlr_tiles.pop_front();
    }
    lock.unlock();
    if(osmlr_filename.empty())
      break;

    //get the local associations
    e.add_tile(osmlr_filename);
  }

  //pass it back
  association.set_value(std::move(e));
}

void add_leftover_associations(const bpt::ptree &pt, std::unordered_map<GraphId, leftovers_t>& leftovers,
                               std::mutex& lock) {

  //something so we can open up a tile builder
  TileFsHierarchy hierarchy(pt.get<std::string>("mjolnir.tile_dir"));

  while(true) {
    // Get leftovers from a tile
    leftovers_t associations;
    lock.lock();
    if (leftovers.size()) {
      associations = std::move(leftovers.begin()->second);
      leftovers.erase(leftovers.begin());
    }
    lock.unlock();
    if(associations.empty())
      break;

    // Write leftovers
    vj::GraphTileBuilder tile_builder(hierarchy, associations.front().first.Tile_Base(), false);
    tile_builder.InitializeTrafficSegments();
    for(const auto& association : associations)
      tile_builder.AddTrafficSegment(association.first, association.second);
    tile_builder.UpdateTrafficSegments();
  }
}

void add_chunks(const bpt::ptree &pt, std::unordered_map<vb::GraphId, chunks_t>& chunks,
                               std::mutex& lock) {
  TileFsHierarchy hierarchy(pt.get<std::string>("mjolnir.tile_dir"));
  while(true) {
    // Get chunks
    lock.lock();
    std::vector<std::pair<vb::GraphId, std::vector<vb::TrafficChunk>>> associated_chunks;
    if (chunks.size()) {
      associated_chunks = std::move(chunks.begin()->second);
      chunks.erase(chunks.begin());
    }
    lock.unlock();
    if(associated_chunks.empty())
      break;

    // Write chunks
    vj::GraphTileBuilder tile_builder(hierarchy, associated_chunks.front().first.Tile_Base(), false);
    tile_builder.InitializeTrafficSegments();
    for(const auto& chunk : associated_chunks) {
      tile_builder.AddTrafficSegments(chunk.first, chunk.second);
    }
    tile_builder.UpdateTrafficSegments();
  }
}

} // anonymous namespace

int main(int argc, char** argv) {
  std::string config, tile_dir;
  unsigned int num_threads = 1;

  bpo::options_description options("valhalla_associate_segments " VERSION "\n"
                                   "\n"
                                   " Usage: valhalla_associate_segments [options]\n"
                                   "\n"
                                   "osmlr associates traffic segment descriptors with a valhalla graph. "
                                   "\n"
                                   "\n");

  options.add_options()
    ("help,h", "Print this help message.")
    ("version,v", "Print the version of this software.")
    ("osmlr-tile-dir,t", bpo::value<std::string>(&tile_dir), "Location of traffic segment tiles.")
    ("concurrency,j", bpo::value<unsigned int>(&num_threads), "Number of threads to use.")
    // positional arguments
    ("config", bpo::value<std::string>(&config), "Valhalla configuration file [required]");


  bpo::positional_options_description pos_options;
  pos_options.add("config", 1);
  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(), vm);
    bpo::notify(vm);
  }
  catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
              << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
              << "\n";
    return EXIT_FAILURE;
  }

  if (vm.count("help") || !vm.count("config")) {
    std::cout << options << "\n";
    return EXIT_SUCCESS;
  }

  if (vm.count("version")) {
    std::cout << "valhalla_associate_segments " << VERSION << "\n";
    return EXIT_SUCCESS;
  }

  if (!vm.count("osmlr-tile-dir")) {
    std::cout << "You must provide a tile directory to read OSMLR tiles from.\n";
    return EXIT_FAILURE;
  }

  //queue up all the work we'll be doing
  std::deque<std::string> osmlr_tiles;
  auto itr = bfs::recursive_directory_iterator(tile_dir);
  auto end = bfs::recursive_directory_iterator();
  for (; itr != end; ++itr) {
    auto dir_entry = *itr;
    if (bfs::is_regular_file(dir_entry)) {
      auto ext = dir_entry.path().extension();
      if (ext == ".osmlr") {
        osmlr_tiles.emplace_back(dir_entry.path().string());
      }
    }
  }
  //std::random_shuffle(osmlr_tiles.begin(), osmlr_tiles.end());

  //configure logging
  vm::logging::Configure({{"type","std_err"},{"color","true"}});

  //parse the config
  bpt::ptree pt;
  bpt::read_json(config.c_str(), pt);

  //fire off some threads to do the work
  LOG_INFO("Associating local traffic segments with " + std::to_string(num_threads) + " threads");
  std::vector<std::shared_ptr<std::thread> > threads(num_threads);
  std::list<std::promise<edge_association> > results;
  std::mutex lock;
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(add_local_associations, std::cref(pt), std::ref(osmlr_tiles),
                                 std::ref(lock), std::ref(results.back())));
  }

  //wait for it to finish
  for (auto& thread : threads)
    thread->join();
  LOG_INFO("Finished");

  // Gather statistics, chunks, and leftovers (associations in a different tile)
  uint32_t success_count = 0;
  uint32_t failure_count = 0;
  uint32_t walk_count = 0;
  uint32_t path_count = 0;
  uint32_t leftover_count = 0;
  uint32_t chunk_count = 0;
  std::unordered_map<vb::GraphId, leftovers_t> leftovers;
  std::unordered_map<vb::GraphId, chunks_t> chunks;
  for (auto& result : results) {
    auto associations = result.get_future().get();
    success_count += associations.success_count();
    failure_count += associations.failure_count();
    walk_count += associations.walk_count();
    path_count += associations.path_count();

    // Leftovers
    for(const auto& association : associations.leftovers()) {
      auto leftover = leftovers.insert({association.first.Tile_Base(), {}}).first;
      leftover->second.push_back(association);
    }
    leftover_count += associations.leftovers().size();

    // Temporary chunks
    using single_chunk_t = std::pair<vb::GraphId, std::vector<vb::TrafficChunk>>;
    for (const auto& association : associations.chunks()) {
      vb::GraphId tileid = association.first.Tile_Base();
      single_chunk_t c = std::make_pair(association.first, association.second);
      if (chunks.find(tileid) == chunks.end()) {
        chunks[tileid] = { c };
      } else {
        chunks[tileid].push_back(c);
      }
    }
    chunk_count += associations.chunks().size();
  }
  LOG_INFO("Success = " + std::to_string(success_count) +
           " Failure = " + std::to_string(failure_count) +
           " Walk = " + std::to_string(walk_count) +
           " Path = " + std::to_string(path_count));
  LOG_INFO("Leftovers = " + std::to_string(leftover_count) +
           " Chunks = " + std::to_string(chunk_count));

  LOG_INFO("Associating neighbouring traffic segments with " + std::to_string(num_threads) + " threads");

  // Write all the leftovers
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(add_leftover_associations, std::cref(pt), std::ref(leftovers),
                                 std::ref(lock)));
  }

  //wait for it to finish
  for (auto& thread : threads)
    thread->join();

  // Write all the chunks
  LOG_INFO("Adding chunks (edge associations to multiple segments) " + std::to_string(num_threads) + " threads");
  for (auto& thread : threads) {
    results.emplace_back();
    thread.reset(new std::thread(add_chunks, std::cref(pt), std::ref(chunks), std::ref(lock)));
  }

  //wait for it to finish
  for (auto& thread : threads)
    thread->join();
  LOG_INFO("Finished");

  LOG_INFO("Finished");

  return EXIT_SUCCESS;
}
