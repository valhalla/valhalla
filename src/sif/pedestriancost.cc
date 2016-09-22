#include "sif/pedestriancost.h"

#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

// Maximum route distances
constexpr uint32_t kMaxDistanceFoot        = 100000; // 100 km
constexpr uint32_t kMaxDistanceWheelchair  = 10000;  // 10 km

// Default speeds
constexpr float kDefaultSpeedFoot          = 5.1f;   // 3.16 MPH
constexpr float kDefaultSpeedWheelchair    = 4.0f;   // 2.5  MPH  TODO

// Penalty to take steps
constexpr float kDefaultStepPenaltyFoot       = 30.0f;   // 30 seconds
constexpr float kDefaultStepPenaltyWheelchair = 600.0f;  // 10 minutes

// Maximum grade30
constexpr uint32_t kDefaultMaxGradeFoot = 90;
constexpr uint32_t kDefaultMaxGradeWheelchair = 12; // Conservative for now...

// Other defaults (not dependent on type)
constexpr float kModeWeight             = 1.5f;   // Favor this mode?
constexpr float kDefaultManeuverPenalty = 5.0f;   // Seconds
constexpr float kDefaultGatePenalty     = 10.0f;  // Seconds
constexpr float kDefaultWalkwayFactor   = 0.9f;   // Slightly favor walkways
constexpr float kDefaultAlleyFactor     = 2.0f;   // Avoid alleys
constexpr float kDefaultDrivewayFactor  = 5.0f;   // Avoid driveways
constexpr float kDefaultFerryCost               = 300.0f; // Seconds
constexpr float kDefaultCountryCrossingCost     = 600.0f; // Seconds
constexpr float kDefaultCountryCrossingPenalty  = 0.0f;   // Seconds

// Maximum distance at the beginning or end of a multimodal route
// that you are willing to travel for this mode.  In this case,
// it is the max walking distance.
constexpr uint32_t kTransitStartEndMaxDistance    = 2415;   // 1.5 miles

// Maximum transfer distance between stops that you are willing
// to travel for this mode.  In this case, it is the max walking
// distance you are willing to walk between transfers.
constexpr uint32_t kTransitTransferMaxDistance   = 805;   // 0.5 miles

// User propensity to use ferries. Range of values from 0 (avoid ferries) to
// 1 (totally comfortable riding on ferries).
constexpr float kDefaultUseFerryFactor = 1.0f;

// Maximum ferry penalty (when use_ferry == 0). Can't make this too large
// since a ferry is sometimes required to complete a route.
constexpr float kMaxFerryPenalty = 8.0f * 3600.0f; // 8 hours

// Minimum and maximum average pedestrian speed (to validate input).
constexpr float kMinPedestrianSpeed = 0.5f;
constexpr float kMaxPedestrianSpeed = 25.0f;
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
   * This method overrides the max_distance with the max_distance_mm per segment
   * distance. An example is a pure walking route may have a max distance of
   * 10000 meters (10km) but for a multi-modal route a lower limit of 5000
   * meters per segment (e.g. from origin to a transit stop or from the last
   * transit stop to the destination).
   */
  virtual void UseMaxMultiModalDistance();

  /**
   * Returns the maximum transfer distance between stops that you are willing
   * to travel for this mode.  In this case, it is the max walking
   * distance you are willing to walk between transfers.
   */
  virtual uint32_t GetMaxTransferDistanceMM();

  /**
   * This method overrides the weight for this mode.  The higher the value
   * the more the mode is favored.
   */
  virtual float GetModeWeight();

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters.
   * @param  edge     Pointer to a directed edge.
   * @param  pred     Predecessor edge information.
   * @param  tile     current tile
   * @param  edgeid   edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const baldr::GraphTile*& tile,
                       const baldr::GraphId& edgeid) const;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges are
   * provided.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           current tile
   * @param  edgeid         edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const EdgeLabel& pred,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::GraphTile*& tile,
                 const baldr::GraphId& edgeid) const;

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
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. Function/functor is also used to filter
   * edges not usable / inaccessible by pedestrians.
   */
   virtual const EdgeFilter GetEdgeFilter() const {
     // Throw back a lambda that checks the access for this type of costing
     auto access_mask = access_mask_;
     return [access_mask](const baldr::DirectedEdge* edge) {
       return !(edge->trans_up() || edge->trans_down() ||
           edge->use() >= Use::kRail ||
          !(edge->forwardaccess() & access_mask));
     };
   }

   virtual const NodeFilter GetNodeFilter() const {
     //throw back a lambda that checks the access for this type of costing
     auto access_mask = access_mask_;
     return [access_mask](const baldr::NodeInfo* node){
       return !(node->access() & access_mask);
     };
   }

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each node's attribution
   * @return Function/functor to be used in filtering out nodes
   */

 private:
  // Type: foot (default), wheelchair, etc.
  std::string type_;
  uint32_t access_mask_;

  // Maximum pedestrian distance.
  uint32_t max_distance_;

  // This is the weight for this mode.  The higher the value the more the
  // mode is favored.
  float mode_weight_;

  // Maximum pedestrian distance in meters for multimodal routes.
  // Maximum distance at the beginning or end of a multimodal route
  // that you are willing to travel for this mode.  In this case,
  // it is the max walking distance.
  uint32_t transit_start_end_max_distance_;

  // Maximum transfer, distance in meters for multimodal routes.
  // Maximum transfer distance between stops that you are willing
  // to travel for this mode.  In this case, it is the max distance
  // you are willing to walk between transfers.
  uint32_t transit_transfer_max_distance_;

  // Minimal surface type usable by the pedestrian type
  Surface minimal_allowed_surface_;

  uint32_t max_grade_;    // Maximum grade (percent).
  float speed_;           // Pedestrian speed.
  float speedfactor_;     // Speed factor for costing. Based on speed.
  float walkway_factor_;  // Factor for favoring walkways and paths.
  float alley_factor_;    // Avoid alleys factor.
  float driveway_factor_; // Avoid driveways factor.
  float step_penalty_;    // Penalty applied to steps/stairs (seconds).
  float gate_penalty_;    // Penalty (seconds) to go through gate
  float maneuver_penalty_;          // Penalty (seconds) when inconsistent names
  float country_crossing_cost_;     // Cost (seconds) to go through toll booth
  float country_crossing_penalty_;  // Penalty (seconds) to go across a country border
  float ferry_cost_;                // Cost (seconds) to exit a ferry
  float ferry_penalty_;             // Penalty (seconds) to enter a ferry
  float ferry_weight_;              // Weighting to apply to ferry edges
};

// Constructor. Parse pedestrian options from property tree. If option is
// not present, set the default.
PedestrianCost::PedestrianCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt, TravelMode::kPedestrian) {
  allow_transit_connections_ = false;

  // Get the type (defaults to foot)
  type_ = pt.get<std::string>("type", "foot");

  // Set type specific defaults, override with URL inputs
  if (type_ == "wheelchair") {
    access_mask_ = kWheelchairAccess;
    max_distance_  = pt.get<uint32_t>("max_distance", kMaxDistanceWheelchair);
    speed_ = pt.get<float>("walking_speed", kDefaultSpeedWheelchair);
    step_penalty_  = pt.get<float>("step_penalty", kDefaultStepPenaltyWheelchair);
    walkway_factor_ = pt.get<float>("walkway_factor", kDefaultWalkwayFactor);
    max_grade_ = pt.get<float>("max_grade", kDefaultMaxGradeWheelchair);
    minimal_allowed_surface_ = Surface::kCompacted;

    // Validate speed (make sure it is in the accepted range)
    if (speed_ < kMinPedestrianSpeed || speed_ > kMaxPedestrianSpeed) {
      LOG_WARN("Outside valid pedestrian speed range " +
                std::to_string(speed_) + ": using default");
      speed_ = kDefaultSpeedWheelchair;
    }
  } else {
    // Assume type = foot
    access_mask_ = kPedestrianAccess;
    max_distance_  = pt.get<uint32_t>("max_distance", kMaxDistanceFoot);
    speed_ = pt.get<float>("walking_speed", kDefaultSpeedFoot);
    step_penalty_  = pt.get<float>("step_penalty", kDefaultStepPenaltyFoot);
    walkway_factor_ = pt.get<float>("walkway_factor", kDefaultWalkwayFactor);
    max_grade_ = pt.get<float>("max_grade", kDefaultMaxGradeFoot);
    minimal_allowed_surface_ = Surface::kPath;

    // Validate speed (make sure it is in the accepted range)
    if (speed_ < kMinPedestrianSpeed || speed_ > kMaxPedestrianSpeed) {
     LOG_WARN("Outside valid pedestrian speed range " +
               std::to_string(speed_) + ": using default");
     speed_ = kDefaultSpeedFoot;
    }
  }

  mode_weight_                    = pt.get<float>("mode_weight", kModeWeight);
  transit_start_end_max_distance_ = pt.get<uint32_t>("transit_start_end_max_distance",
                                                     kTransitStartEndMaxDistance);
  transit_transfer_max_distance_  = pt.get<uint32_t>("transit_transfer_max_distance",
                                                     kTransitTransferMaxDistance);
  alley_factor_                   = pt.get<float>("alley_factor", kDefaultAlleyFactor);
  driveway_factor_                = pt.get<float>("driveway_factor", kDefaultDrivewayFactor);

  gate_penalty_                   = pt.get<float>("gate_penalty", kDefaultGatePenalty);
  maneuver_penalty_               = pt.get<float>("maneuver_penalty",
                                                  kDefaultManeuverPenalty);
  country_crossing_cost_          = pt.get<float>("country_crossing_cost",
                                                  kDefaultCountryCrossingCost);
  country_crossing_penalty_       = pt.get<float>("country_crossing_penalty",
                                                  kDefaultCountryCrossingPenalty);

  // Set the cost (seconds) to enter a ferry (only apply entering since
  // a route must exit a ferry (except artificial test routes ending on
  // a ferry!)
  ferry_cost_ = pt.get<float>("ferry_cost", kDefaultFerryCost);

  // Modify ferry penalty and edge weighting based on use_ferry factor
  float use_ferry = pt.get<float>("use_ferry", kDefaultUseFerryFactor);
  if (use_ferry < 0.5f) {
    // Penalty goes from max at use_ferry = 0 to 0 at use_ferry = 0.5
    float w = 1.0f - ((0.5f - use_ferry) * 2.0f);
    ferry_penalty_ = static_cast<uint32_t>(kMaxFerryPenalty * (1.0f - w));

    // Double the cost at use_ferry == 0, progress to 1.0 at use_ferry = 0.5
    ferry_weight_ = 1.0f + w;
  } else {
    // Add a ferry weighting factor to influence cost along ferries to make
    // them more favorable if desired rather than driving. No ferry penalty.
    // Half the cost at use_ferry == 1, progress to 1.0 at use_ferry = 0.5
    ferry_penalty_ = 0.0f;
    ferry_weight_  = 1.0f - (use_ferry - 0.5f);
  }

  // Set the speed factor (to avoid division in costing)
  speedfactor_ = (kSecPerHour * 0.001f) / speed_;
}

// Destructor
PedestrianCost::~PedestrianCost() {
}

// This method overrides the max_distance with the max_distance_mm per segment
// distance. An example is a pure walking route may have a max distance of
// 10000 meters (10km) but for a multi-modal route a lower limit of 5000
// meters per segment (e.g. from origin to a transit stop or from the last
// transit stop to the destination).
void PedestrianCost::UseMaxMultiModalDistance() {
  max_distance_ = transit_start_end_max_distance_;
}

// Returns the maximum transfer distance between stops that you are willing
// to travel for this mode.  In this case, it is the max walking
// distance you are willing to walk between transfers.
uint32_t PedestrianCost::GetMaxTransferDistanceMM() {
  return transit_transfer_max_distance_;
}

// This method overrides the weight for this mode.  The higher the value
// the more the mode is favored.
float PedestrianCost::GetModeWeight() {
  return mode_weight_;
}

// Check if access is allowed on the specified edge. Disallow if no
// access for this pedestrian type, if surface type exceeds (worse than)
// the minimum allowed surface type, or if max grade is exceeded.
// Disallow edges where max. distance will be exceeded.
bool PedestrianCost::Allowed(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const baldr::GraphTile*& tile,
                             const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  if (!(edge->forwardaccess() & access_mask_) ||
       (edge->surface() > minimal_allowed_surface_) ||
 //      (edge->max_up_slope() > max_grade_ || edge->max_down_slope() > max_grade_) ||
      ((pred.path_distance() + edge->length()) > max_distance_)) {
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
               const EdgeLabel& pred,
               const baldr::DirectedEdge* opp_edge,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  // Do not check max walking distance and assume we are not allowing
  // transit connections. Assume this method is never used in
  // multimodal routes).
  if (!(opp_edge->forwardaccess() & access_mask_) ||
       (opp_edge->surface() > minimal_allowed_surface_) ||
 //      (opp_edge->max_up_slope() > max_grade_ || opp_edge->max_down_slope() > max_grade_) ||
        opp_edge->use() == Use::kTransitConnection) {
    return false;
  }
  return true;
}

// Check if access is allowed at the specified node.
bool PedestrianCost::Allowed(const baldr::NodeInfo* node) const {
  return (node->access() & access_mask_);
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost PedestrianCost::EdgeCost(const baldr::DirectedEdge* edge,
                              const uint32_t density) const {
  // Ferries are a special case - they use the ferry speed (stored on the edge)
  if (edge->use() == Use::kFerry) {
    float sec = edge->length() * (kSecPerHour * 0.001f) /
            static_cast<float>(edge->speed());
    return { sec * ferry_weight_, sec };
  }

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

  // Penalty through gates and border control.
  float seconds = 0.0f;
  float penalty = 0.0f;
  if (node->type() == NodeType::kBorderControl) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  } else if (node->type() == NodeType::kGate) {
    penalty += gate_penalty_;
  }

  if ((pred.use() != Use::kFerry && edge->use() == Use::kFerry)) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }

  // Slight maneuver penalty
  if (!node->name_consistency(pred.opp_local_idx(), edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  // Costs for crossing an intersection.
  // TODO - do we want any maneuver penalty?
  uint32_t idx = pred.opp_local_idx();
  if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
    seconds += edge->stopimpact(idx);
  }
  return { seconds + penalty, seconds };
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

  // Penalty through gates and border control.
  float seconds = 0.0f;
  float penalty = 0.0f;
  if (node->type() == NodeType::kBorderControl) {
    seconds += country_crossing_cost_;
    penalty += country_crossing_penalty_;
  } else if (node->type() == NodeType::kGate) {
    penalty += gate_penalty_;
  }

  if (opp_edge->use() != Use::kFerry && opp_pred_edge->use() == Use::kFerry) {
    seconds += ferry_cost_;
    penalty += ferry_penalty_;
  }

  // Slight maneuver penalty
  if (!node->name_consistency(idx, opp_pred_edge->localedgeidx())) {
    penalty += maneuver_penalty_;
  }

  // Costs for crossing an intersection.
  // TODO - do we want any maneuver penalty?
  if (opp_pred_edge->edge_to_right(idx) && opp_pred_edge->edge_to_left(idx)) {
    seconds += opp_pred_edge->stopimpact(idx);
  }
  return { seconds + penalty, seconds };
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
