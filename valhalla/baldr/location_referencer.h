#ifndef __VALHALLA_BALDR_LOCATION_REFERENCER_H__
#define __VALHALLA_BALDR_LOCATION_REFERENCER_H__

#include "baldr/graphreader.h"
#include "baldr/pathlocation.h"
#include "midgard/openlr.h"
#include "thor/pathinfo.h"
#include "thor/astar.h"

namespace valhalla {
namespace baldr {

// Matched edge
struct EdgeMatch {
  GraphId edgeid;
  uint32_t length;
  float start_pct;
  float end_pct;
};


namespace {

// Distance tolerance (meters) for node searching. This value allows some
// tolerance to account for data edits.
constexpr float kNodeDistanceTolerance = 20.0;

// Distance tolerance (meters) for searching along an edge. This value allows
// some tolerance to account for data edits.
constexpr uint32_t kEdgeDistanceTolerance = 20.0;

// 10 meter length matching tolerance.
// TODO - should this be based on segment length so that short segments have
// less tolerance?
constexpr uint32_t kLengthToleranceMetres = 50;

// Bearing tolerance in degrees
constexpr uint16_t kBearingTolerance = 6; // std::ceil(11.25/2);


enum class MatchType : uint8_t {
  kWalk = 0,
  kShortestPath = 1
};

struct CandidateEdge {
  PathLocation::PathEdge edge;
  float distance;

  CandidateEdge()
    : edge({}, 0.0f, {}, 1.0f),
      distance(0.0f) {
  }

  CandidateEdge(const PathLocation::PathEdge& e, const float d)
      : edge(e),
        distance(d) {
  }
};

struct edge_association {
  explicit edge_association(baldr::GraphReader &graphreader);

  std::vector<EdgeMatch> match_edges(const midgard::OpenLR::TwoPointLinearReference &locRef);

private:
  std::vector<CandidateEdge> candidate_edges(bool origin,
                        const midgard::PointLL &lrp,
                        const double bearing,
                        const uint8_t level);
  std::vector<EdgeMatch> walk(const baldr::GraphId& segment_id,
                    const uint32_t segment_length,
                    const midgard::OpenLR::TwoPointLinearReference &locRef);

  baldr::GraphReader &m_reader;
  sif::TravelMode m_travel_mode;
  std::shared_ptr<thor::AStarPathAlgorithm> m_path_algo;
  std::shared_ptr<sif::DynamicCost> m_costing;

};
}

struct LocationReferencer {

  LocationReferencer(baldr::GraphReader &graphreader);

  std::vector<EdgeMatch> match(const midgard::OpenLR::TwoPointLinearReference &locref);

private:
  edge_association association;
};

} // namespace baldr
} // namespace valhalla

#endif // __VALHALLA_BALDR_LOCATION_REFERENCER_H__