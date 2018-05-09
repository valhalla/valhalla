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