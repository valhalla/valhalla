// -*- mode: c++ -*-

#include <valhalla/sif/dynamiccost.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::sif;
using namespace valhalla::baldr;


namespace {
// Maximum walking distance for any walking leg of a multimodal route.
// From origin/destination to any transit stop, parking, etc. or between
// any stops
constexpr uint32_t kMaxModeDistance    = 3000;   // 3 km

// Maximum distance of a walking route
constexpr uint32_t kMaxDistance2        = 100000; // 100 km

constexpr float kDefaultWalkingSpeed   = 5.1f;   // 3.16 MPH
constexpr float kDefaultWalkwayFactor  = 0.9f;   // Slightly favor walkways
constexpr float kDefaultAlleyFactor    = 2.0f;   // Avoid alleys
constexpr float kDefaultDrivewayFactor = 5.0f;   // Avoid driveways
constexpr float kDefaultStepPenalty    = 30.0f;  // 30 seconds

// Minimum and maximum average walking speed (to validate input).
constexpr float kMinWalkingSpeed = 0.5f;
constexpr float kMaxWalkingSpeed = 25.0f;
}


class PedestrianCost : public DynamicCost {
 public:
  // Constructor. Parse pedestrian options from property tree. If option is
  // not present, set the default.
  PedestrianCost(const boost::property_tree::ptree& pt)
      : DynamicCost(pt, TravelMode::kPedestrian) {
    allow_transit_connections_ = false;
    max_distance_      = pt.get<uint32_t>("max_distance", kMaxDistance2);
    max_mode_distance_ = pt.get<uint32_t>("max_mode_distance", kMaxModeDistance);
    walking_speed_     = pt.get<float>("walking_speed", kDefaultWalkingSpeed);
    walkway_factor_    = pt.get<float>("walkway_factor", kDefaultWalkwayFactor);
    alley_factor_      = pt.get<float>("alley_factor_", kDefaultAlleyFactor);
    driveway_factor_   = pt.get<float>("driveway_factor", kDefaultDrivewayFactor);
    step_penalty_      = pt.get<float>("step_penalty", kDefaultStepPenalty);

    // Validate speed (make sure it is in the accepted range)
    if (walking_speed_ < kMinWalkingSpeed || walking_speed_ > kMaxWalkingSpeed) {
      LOG_WARN("Outside valid walking speed range " +
               std::to_string(walking_speed_) + ": using default");
      walking_speed_ = kDefaultWalkingSpeed;
    }

    // Set the speed factor (to avoid division in costing)
    speedfactor_ = (kSecPerHour * 0.001f) / walking_speed_;
  }

  bool Allowed(const DirectedEdge* edge,
               const EdgeLabel& pred) const {
    // Disallow if no pedestrian access, surface marked as impassible,
    // edge is not-thru and we are far from destination, or if max
    // walking distance is exceeded.
    if (!(edge->forwardaccess() & kPedestrianAccess) ||
        (edge->surface() == Surface::kImpassable) ||
        (edge->not_thru() && pred.distance() > not_thru_distance_) ||
        ((pred.walking_distance() + edge->length()) > max_distance_)) {
      return false;
    }

    // Check for U-turn (if predecessor mode is also pedestrian)
    if (pred.mode() == TravelMode::kPedestrian &&
        pred.opp_local_idx() == edge->localedgeidx()) {
      return false;
    }

    // Disallow transit connections (except when set for multi-modal routes)
    if (!allow_transit_connections_ && edge->use() == Use::kTransitConnection) {
      return false;
    }
    return true;
  }

  bool AllowedReverse(const DirectedEdge* edge,
                      const DirectedEdge* opp_edge,
                      const DirectedEdge* opp_pred_edge) const
  {
    // Disallow if no pedestrian access, surface marked as impassible, Uturn,
    // or edge is not-thru (no need to check distance from destination since
    // the search is heading out of any not_thru regions). Do not check max
    // walking distance and assume we are not allowing transit connections.
    // Assume this method is never used in multimodal routes).
    if (!(opp_edge->forwardaccess() & kPedestrianAccess) ||
        (opp_pred_edge->localedgeidx() == edge->localedgeidx()) ||
        opp_edge->surface() == Surface::kImpassable ||
        edge->not_thru() || opp_edge->use() == Use::kTransitConnection) {
      return false;
    }
    return true;
  }

  bool Allowed(const NodeInfo* node) const
  {
    return (node->access() & kPedestrianAccess);
  }

  Cost EdgeCost(const DirectedEdge* edge,
                const uint32_t density) const
  {
    float length = edge->length();
    return { length, length };
  }

  // Disable astar
  float AStarCostFactor() const {
    return 0.f;
  }

  virtual const EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const DirectedEdge* edge){
      return edge->trans_up() || edge->trans_down() ||
          edge->use() >= Use::kRail ||
          !(edge->forwardaccess() & kPedestrianAccess);
    };
  }

 private:
  // Maximum walking distance
  uint32_t max_distance_;

  // Maximum walking distance in meters for multimodal routes. This is the maximum
  // distance for any single walking portion of the route.
  uint32_t max_mode_distance_;

  // Walking speed (default to 5.1 km / hour)
  float walking_speed_;

  // Speed factor for costing. Based on walking speed.
  float speedfactor_;

  // Factor for favoring walkways and paths. Default to 0.9.
  float walkway_factor_;

  // Avoid alleys. Default to 2.0. Double the cost to traverse an alley.
  float alley_factor_;

  // Avoid driveways. Default to 5.0. 5x cost to traverse.
  float driveway_factor_;

  // Avoid stairs/steps. This is a fixed cost in seconds.
  float step_penalty_;
};


cost_ptr_t CreatePedestrianCost(const boost::property_tree::ptree& config) {
  return std::make_shared<PedestrianCost>(config);
}
