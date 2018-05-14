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

// Maximum distance to edge for nearest-neighbour search
constexpr uint32_t kEdgeDistanceTolerance = 20.0;

// OpenLR limits distance between coordinates to 15000m in the spec
// The binary format stores distances between points as a single byte
// with reduced precision of 15000m/256 ~= 58.6m.  Values stored
// in the descriptor are std::floor(length/58.6).
// Returned matches therefore could be +/- 58.6 in length and still
// be correct due to precision limitations
constexpr double kLengthToleranceMetres = 58.6;  // std::round(15000/256)

// Bearings are stored as 5 bits referring to a segment.  Each segment
// is thus 360/2^5 ~= 11.25 degrees
constexpr double kSegmentSize = 11.25/2;

}

struct LocationReferencer {

  LocationReferencer(baldr::GraphReader &graphreader);

  std::vector<EdgeMatch> match(const midgard::OpenLR::TwoPointLinearReference &locref);

private:
  baldr::GraphReader &m_reader;
  sif::TravelMode m_travel_mode;
  std::shared_ptr<thor::AStarPathAlgorithm> m_path_algo;
  std::shared_ptr<sif::DynamicCost> m_costing;
};

} // namespace baldr
} // namespace valhalla

#endif // __VALHALLA_BALDR_LOCATION_REFERENCER_H__