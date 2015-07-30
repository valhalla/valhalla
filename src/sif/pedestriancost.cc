#include "sif/pedestriancost.h"

#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

// Maximum walking distance for any walking leg of a multimodal route.
// From origin/destination to any transit stop, parking, etc. or between
// any stops
constexpr uint32_t kMaxModeDistance    = 3000;   // 3 km

// Maximum distance of a walking route
constexpr uint32_t kMaxDistance        = 100000; // 100 km

constexpr float kDefaultWalkingSpeed   = 5.1f;   // 3.16 MPH
constexpr float kDefaultWalkwayFactor  = 0.9f;   // Slightly favor walkways
constexpr float kDefaultAlleyFactor    = 2.0f;   // Avoid alleys
constexpr float kDefaultDrivewayFactor = 5.0f;   // Avoid driveways
constexpr float kDefaultStepPenalty    = 30.0f;  // 30 seconds

// Minimum and maximum average walking speed (to validate input).
constexpr float kMinWalkingSpeed = 0.5f;
constexpr float kMaxWalkingSpeed = 25.0f;
}

/**
 * Derived class providing dynamic edge costing for pedestrian routes.
 */
class PedestrianCost : public DynamicCost {
 public:
  /**
   * Constructor. Configuration / options for pedestrian costing are provided
   * via a property tree (JSON).
   * @param  pt  Property tree with configuration/options.
   */
  PedestrianCost(const boost::property_tree::ptree& pt);

  virtual ~PedestrianCost();

  /**
   * This method overrides the max_distance with the multi-modal per segment
   * distance. An example is a pure walking route may have a max distance of
   * 10000 meters (10km) but for a multi-modal route a lower limit of 5000
   * meters per segment (e.g. from origin to a transit stop or from the last
   * transit stop to the destination).
   */
  virtual void UseMaxModeDistance();

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge  Pointer to a directed edge.
   * @param  pred  Predecessor edge information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges are
   * provided.
   * @param  edge  Pointer to a directed edge.
   * @param  opp_edge  Pointer to the opposing directed edge.
   * @param  opp_pred_edge  Pointer to the opposing directed edge to the
   *                        predecessor.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::DirectedEdge* opp_pred_edge) const;

  /**
   * Checks if access is allowed for the provided node. Node access can
   * be restricted if bollards or gates are present.
   * @param  edge  Pointer to node information.
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::NodeInfo* node) const;

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge  Pointer to a directed edge.
   * @param   density  Relative road density.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const uint32_t density) const;

  /**
   * Returns the cost to make the transition from the predecessor edge.
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  edge  Directed edge (the to edge)
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  Predecessor edge information.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCost(const baldr::DirectedEdge* edge,
                              const baldr::NodeInfo* node,
                              const EdgeLabel& pred) const;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  opp_edge  Pointer to the opposing directed edge - this is the
   *                   "from" or predecessor edge in the transition.
   * @param  opp_pred_edge  Pointer to the opposing directed edge to the
   *                        predecessor. This is the "to" edge.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                              const baldr::NodeInfo* node,
                              const baldr::DirectedEdge* opp_edge,
                              const baldr::DirectedEdge* opp_pred_edge) const;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each edges attribution
   * @return Function/functor to be used in filtering out edges
   */
  virtual const EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    return [](const baldr::DirectedEdge* edge){
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

// Constructor. Parse pedestrian options from property tree. If option is
// not present, set the default.
PedestrianCost::PedestrianCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt, TravelMode::kPedestrian) {
  allow_transit_connections_ = false;
  max_distance_      = pt.get<uint32_t>("max_distance", kMaxDistance);
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

// Destructor
PedestrianCost::~PedestrianCost() {
}

// This method overrides the max_distance with the multi-modal per segment
// distance. An example is a pure walking route may have a max distance of
// 10000 meters (10km) but for a multi-modal route a lower limit of 5000
//  meters per segment (e.g. from origin to a transit stop or from the last
//  transit stop to the destination).
void PedestrianCost::UseMaxModeDistance() {
  max_distance_ = max_mode_distance_;
}


// Check if access is allowed on the specified edge. Disallow if no pedestrian
// access. Disallow Uturns or entering not-thru edges except near the
// destination. Do not allow if surface is impassable. Disallow edges
// where max. walking distance will be exceeded.
bool PedestrianCost::Allowed(const baldr::DirectedEdge* edge,
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

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool PedestrianCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const baldr::DirectedEdge* opp_edge,
               const baldr::DirectedEdge* opp_pred_edge) const {
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

// Check if access is allowed at the specified node.
bool PedestrianCost::Allowed(const baldr::NodeInfo* node) const {
  return (node->access() & kPedestrianAccess);
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost PedestrianCost::EdgeCost(const baldr::DirectedEdge* edge,
                              const uint32_t density) const {
  // Slightly favor walkways/paths and penalize alleys and driveways.
  float sec = edge->length() * speedfactor_;
  if (edge->use() == Use::kFootway) {
    return { sec * walkway_factor_, sec };
  } else if (edge->use() == Use::kAlley) {
    return { sec * alley_factor_, sec };
  } else if (edge->use() == Use::kDriveway) {
    return { sec * driveway_factor_, sec };
  } else {
    return { sec, sec };
  }
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost PedestrianCost::TransitionCost(const baldr::DirectedEdge* edge,
                               const baldr::NodeInfo* node,
                               const EdgeLabel& pred) const {
  // Special cases: fixed penalty for steps/stairs
  if (edge->use() == Use::kSteps) {
    return { step_penalty_, 0.0f };
  }

  // No cost from transit connection
  if (pred.use() == Use::kTransitConnection) {
    return { 0.0f, 0.0f };
  }

  // Costs for crossing an intersection.
  // TODO - do we want any maneuver penalty?
  uint32_t idx = pred.opp_local_idx();
  if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
    float sec = edge->stopimpact(idx);
    return { sec, sec };
  } else {
    return { 0.0f, 0.0f };
  }
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost PedestrianCost::TransitionCostReverse(const uint32_t idx,
                     const baldr::NodeInfo* node,
                     const baldr::DirectedEdge* opp_edge,
                     const baldr::DirectedEdge* opp_pred_edge) const {
  // Special cases: fixed penalty for steps/stairs
  if (opp_pred_edge->use() == Use::kSteps) {
    return { step_penalty_, 0.0f };
  }

  // Costs for crossing an intersection.
  // TODO - do we want any maneuver penalty?
  if (opp_pred_edge->edge_to_right(idx) && opp_pred_edge->edge_to_left(idx)) {
    float sec = opp_pred_edge->stopimpact(idx);
    return { sec, sec };
  } else {
    return { 0.0f, 0.0f };
  }
}

// Get the cost factor for A* heuristics. This factor is multiplied
// with the distance to the destination to produce an estimate of the
// minimum cost to the destination. The A* heuristic must underestimate the
// cost to the destination. So a time based estimate based on speed should
// assume the maximum speed is used to the destination such that the time
// estimate is less than the least possible time along roads.
float PedestrianCost::AStarCostFactor() const {
  // Use the factor to favor walkways/paths if < 1.0f
  return (walkway_factor_ < 1.0f) ? walkway_factor_ * speedfactor_ : speedfactor_;
}

cost_ptr_t CreatePedestrianCost(const boost::property_tree::ptree& config) {
  return std::make_shared<PedestrianCost>(config);
}

}
}
