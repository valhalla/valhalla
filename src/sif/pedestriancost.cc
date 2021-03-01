#include "sif/pedestriancost.h"
#include "baldr/accessrestriction.h"
#include "baldr/graphconstants.h"
#include "midgard/constants.h"
#include "midgard/util.h"
#include "proto/options.pb.h"
#include "proto_conversions.h"
#include "sif/costconstants.h"

#ifdef INLINE_TEST
#include "test.h"
#include "worker.h"
#include <random>
#endif

using namespace valhalla::midgard;
using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {

// Base transition costs
// TODO - can we define these in dynamiccost.h and override here if they differ?
constexpr float kDefaultDestinationOnlyPenalty = 600.0f; // Seconds
constexpr float kDefaultManeuverPenalty = 5.0f;          // Seconds
constexpr float kDefaultAlleyPenalty = 5.0f;             // Seconds
constexpr float kDefaultGatePenalty = 10.0f;             // Seconds
constexpr float kDefaultTollBoothCost = 15.0f;           // Seconds
constexpr float kDefaultFerryCost = 300.0f;              // Seconds
constexpr float kDefaultCountryCrossingCost = 600.0f;    // Seconds
constexpr float kDefaultCountryCrossingPenalty = 0.0f;   // Seconds
constexpr float kDefaultBssCost = 120.0f;                // Seconds
constexpr float kDefaultBssPenalty = 0.0f;               // Seconds
constexpr float kDefaultServicePenalty = 0.0f;           // Seconds

// Maximum route distances
constexpr uint32_t kMaxDistanceFoot = 100000;      // 100 km
constexpr uint32_t kMaxDistanceWheelchair = 10000; // 10 km

// Default speeds
constexpr float kDefaultSpeedFoot = 5.1f;       // 3.16 MPH
constexpr float kDefaultSpeedWheelchair = 4.0f; // 2.5  MPH  TODO

// Penalty to take steps
constexpr float kDefaultStepPenaltyFoot = 30.0f;        // 30 seconds
constexpr float kDefaultStepPenaltyWheelchair = 600.0f; // 10 minutes

// Maximum grade30
constexpr uint32_t kDefaultMaxGradeFoot = 90;
constexpr uint32_t kDefaultMaxGradeWheelchair = 12; // Conservative for now...

// Other defaults (not dependent on type)
constexpr uint8_t kDefaultMaxHikingDifficulty = 1; // T1 (kHiking)
constexpr float kModeFactor = 1.5f;                // Favor this mode?
constexpr float kDefaultWalkwayFactor = 1.0f;      // Neutral value for walkways
constexpr float kDefaultSideWalkFactor = 1.0f;     // Neutral value for sidewalks
constexpr float kDefaultAlleyFactor = 2.0f;        // Avoid alleys
constexpr float kDefaultDrivewayFactor = 5.0f;     // Avoid driveways
constexpr float kDefaultUseFerry = 1.0f;
constexpr float kDefaultUseTracks = 0.5f;        // Factor between 0 and 1
constexpr float kDefaultUseLivingStreets = 0.6f; // Factor between 0 and 1

// Maximum distance at the beginning or end of a multimodal route
// that you are willing to travel for this mode.  In this case,
// it is the max walking distance.
constexpr uint32_t kTransitStartEndMaxDistance = 2415; // 1.5 miles

// Maximum transfer distance between stops that you are willing
// to travel for this mode.  In this case, it is the max walking
// distance you are willing to walk between transfers.
constexpr uint32_t kTransitTransferMaxDistance = 805; // 0.5 miles

// Avoid roundabouts
constexpr float kRoundaboutFactor = 2.0f;

// How much to avoid generic service roads.
constexpr float kDefaultServiceFactor = 1.0f;

// Minimum and maximum average pedestrian speed (to validate input).
constexpr float kMinPedestrianSpeed = 0.5f;
constexpr float kMaxPedestrianSpeed = 25.0f;

// Crossing penalties. TODO - may want to lower stop impact when
// 2 cycleways or walkways cross
constexpr uint32_t kCrossingCosts[] = {0, 0, 1, 1, 2, 3, 5, 15};

constexpr float kMinFactor = 0.1f;
constexpr float kMaxFactor = 100000.0f;

// Valid ranges and defaults
constexpr ranged_default_t<uint32_t> kMaxDistanceWheelchairRange{0, kMaxDistanceWheelchair,
                                                                 kMaxDistanceFoot};
constexpr ranged_default_t<uint32_t> kMaxDistanceFootRange{0, kMaxDistanceFoot, kMaxDistanceFoot};

constexpr ranged_default_t<float> kSpeedWheelchairRange{kMinPedestrianSpeed, kDefaultSpeedWheelchair,
                                                        kMaxPedestrianSpeed};
constexpr ranged_default_t<float> kSpeedFootRange{kMinPedestrianSpeed, kDefaultSpeedFoot,
                                                  kMaxPedestrianSpeed};

constexpr ranged_default_t<float> kStepPenaltyWheelchairRange{0, kDefaultStepPenaltyWheelchair,
                                                              kMaxPenalty};
constexpr ranged_default_t<float> kStepPenaltyFootRange{0, kDefaultStepPenaltyFoot, kMaxPenalty};

constexpr ranged_default_t<uint32_t> kMaxGradeWheelchairRange{0, kDefaultMaxGradeWheelchair,
                                                              kDefaultMaxGradeFoot};
constexpr ranged_default_t<uint32_t> kMaxGradeFootRange{0, kDefaultMaxGradeFoot,
                                                        kDefaultMaxGradeFoot};

// Other valid ranges and defaults (not dependent on type)
constexpr ranged_default_t<uint8_t> kMaxHikingDifficultyRange{0, kDefaultMaxHikingDifficulty, 6};
constexpr ranged_default_t<float> kModeFactorRange{kMinFactor, kModeFactor, kMaxFactor};
constexpr ranged_default_t<float> kManeuverPenaltyRange{kMinFactor, kDefaultManeuverPenalty,
                                                        kMaxPenalty};
constexpr ranged_default_t<float> kDestinationOnlyPenaltyRange{0, kDefaultDestinationOnlyPenalty,
                                                               kMaxPenalty};
constexpr ranged_default_t<float> kAlleyPenaltyRange{0.0f, kDefaultAlleyPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kGatePenaltyRange{0.0f, kDefaultGatePenalty, kMaxPenalty};
constexpr ranged_default_t<float> kWalkwayFactorRange{kMinFactor, kDefaultWalkwayFactor, kMaxFactor};
constexpr ranged_default_t<float> kSideWalkFactorRange{kMinFactor, kDefaultSideWalkFactor,
                                                       kMaxFactor};
constexpr ranged_default_t<float> kAlleyFactorRange{kMinFactor, kDefaultAlleyFactor, kMaxFactor};
constexpr ranged_default_t<float> kDrivewayFactorRange{kMinFactor, kDefaultDrivewayFactor,
                                                       kMaxFactor};
constexpr ranged_default_t<float> kFerryCostRange{0, kDefaultFerryCost, kMaxPenalty};
constexpr ranged_default_t<float> kCountryCrossingCostRange{0, kDefaultCountryCrossingCost,
                                                            kMaxPenalty};
constexpr ranged_default_t<float> kCountryCrossingPenaltyRange{0, kDefaultCountryCrossingPenalty,
                                                               kMaxPenalty};
constexpr ranged_default_t<uint32_t> kTransitStartEndMaxDistanceRange{0, kTransitStartEndMaxDistance,
                                                                      100000}; // Max 100k
constexpr ranged_default_t<uint32_t> kTransitTransferMaxDistanceRange{0, kTransitTransferMaxDistance,
                                                                      50000}; // Max 50k
constexpr ranged_default_t<float> kUseFerryRange{0, kDefaultUseFerry, 1.0f};

constexpr ranged_default_t<float> kBSSCostRange{0, kDefaultBssCost, kMaxPenalty};
constexpr ranged_default_t<float> kBSSPenaltyRange{0, kDefaultBssPenalty, kMaxPenalty};
constexpr ranged_default_t<float> kUseTracksRange{0.f, kDefaultUseTracks, 1.0f};
constexpr ranged_default_t<float> kUseLivingStreetsRange{0.f, kDefaultUseLivingStreets, 1.0f};
constexpr ranged_default_t<float> kServicePenaltyRange{0.0f, kDefaultServicePenalty, kMaxPenalty};
constexpr ranged_default_t<float> kServiceFactorRange{kMinFactor, kDefaultServiceFactor, kMaxFactor};

constexpr float kSacScaleSpeedFactor[] = {
    1.0f,  // kNone
    1.11f, // kHiking (~90% speed)
    1.25f, // kMountainHiking (80% speed)
    1.54f, // kDemandingMountainHiking (~65% speed)
    2.5f,  // kAlpineHiking (40% speed)
    4.0f,  // kDemandingAlpineHiking (25% speed)
    6.67f  // kDifficultAlpineHiking (~15% speed)
};

constexpr float kSacScaleCostFactor[] = {
    0.0f,  // kNone
    0.25f, // kHiking
    0.75f, // kMountainHiking
    1.25f, // kDemandingMountainHiking
    2.0f,  // kAlpineHiking
    2.5f,  // kDemandingAlpineHiking
    3.0f   // kDifficultAlpineHiking
};

} // namespace

/**
 * Derived class providing dynamic edge costing for pedestrian routes.
 */
class PedestrianCost : public DynamicCost {
public:
  /**
   * Construct pedestrian costing. Pass in cost type and costing_options using protocol buffer(pbf).
   * @param  costing specified costing type.
   * @param  costing_options pbf with request costing_options.
   */
  PedestrianCost(const CostingOptions& costing_options);

  // virtual destructor
  virtual ~PedestrianCost() {
  }

  /**
   * Does the costing method allow multiple passes (with relaxed hierarchy
   * limits).
   * @return  Returns true if the costing model allows multiple passes.
   */
  virtual bool AllowMultiPass() const override {
    return true;
  }

  /**
   * This method overrides the max_distance with the max_distance_mm per segment
   * distance. An example is a pure walking route may have a max distance of
   * 10000 meters (10km) but for a multi-modal route a lower limit of 5000
   * meters per segment (e.g. from origin to a transit stop or from the last
   * transit stop to the destination).
   */
  virtual void UseMaxMultiModalDistance() override {
    max_distance_ = transit_start_end_max_distance_;
  }

  /**
   * Returns the maximum transfer distance between stops that you are willing
   * to travel for this mode.  In this case, it is the max walking
   * distance you are willing to walk between transfers.
   */
  virtual uint32_t GetMaxTransferDistanceMM() override {
    return transit_transfer_max_distance_;
  }

  /**
   * This method overrides the factor for this mode.  The higher the value
   * the more the mode is favored.
   */
  virtual float GetModeFactor() override {
    return mode_factor_;
  }

  /**
   * Checks if access is allowed for the provided directed edge.
   * This is generally based on mode of travel and the access modes
   * allowed on the edge. However, it can be extended to exclude access
   * based on other parameters such as conditional restrictions and
   * conditional access that can depend on time and travel mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the directed edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return Returns true if access is allowed, false if not.
   */
  virtual bool Allowed(const baldr::DirectedEdge* edge,
                       const EdgeLabel& pred,
                       const graph_tile_ptr& tile,
                       const baldr::GraphId& edgeid,
                       const uint64_t current_time,
                       const uint32_t tz_index,
                       uint8_t& restriction_idx) const override;

  /**
   * Checks if access is allowed for an edge on the reverse path
   * (from destination towards origin). Both opposing edges (current and
   * predecessor) are provided. The access check is generally based on mode
   * of travel and the access modes allowed on the edge. However, it can be
   * extended to exclude access based on other parameters such as conditional
   * restrictions and conditional access that can depend on time and travel
   * mode.
   * @param  edge           Pointer to a directed edge.
   * @param  pred           Predecessor edge information.
   * @param  opp_edge       Pointer to the opposing directed edge.
   * @param  tile           Current tile.
   * @param  edgeid         GraphId of the opposing edge.
   * @param  current_time   Current time (seconds since epoch). A value of 0
   *                        indicates the route is not time dependent.
   * @param  tz_index       timezone index for the node
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                              const EdgeLabel& pred,
                              const baldr::DirectedEdge* opp_edge,
                              const graph_tile_ptr& tile,
                              const baldr::GraphId& opp_edgeid,
                              const uint64_t current_time,
                              const uint32_t tz_index,
                              uint8_t& restriction_idx) const override;

  /**
   * Only transit costings are valid for this method call, hence we throw
   * @param edge
   * @param departure
   * @param curr_time
   * @return
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge*,
                        const baldr::TransitDeparture*,
                        const uint32_t) const override {
    throw std::runtime_error("PedestrianCost::EdgeCost does not support transit edges");
  }

  bool IsClosed(const baldr::DirectedEdge*, const graph_tile_ptr&) const override {
    return false;
  }

  /**
   * Get the cost to traverse the specified directed edge. Cost includes
   * the time (seconds) to traverse the edge.
   * @param  edge      Pointer to a directed edge.
   * @param  tile      Current tile.
   * @param  seconds   Time of week in seconds.
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const graph_tile_ptr& tile,
                        const uint32_t seconds,
                        uint8_t& flow_sources) const override;

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
                              const EdgeLabel& pred) const override;

  /**
   * Returns the cost to make the transition from the predecessor edge
   * when using a reverse search (from destination towards the origin).
   * Defaults to 0. Costing models that wish to include edge transition
   * costs (i.e., intersection/turn costs) must override this method.
   * @param  idx   Directed edge local index
   * @param  node  Node (intersection) where transition occurs.
   * @param  pred  the opposing current edge in the reverse tree.
   * @param  edge  the opposing predecessor in the reverse tree
   * @param  has_measured_speed Do we have any of the measured speed types set?
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost TransitionCostReverse(const uint32_t idx,
                                     const baldr::NodeInfo* node,
                                     const baldr::DirectedEdge* pred,
                                     const baldr::DirectedEdge* edge,
                                     const bool /*has_measured_speed*/) const override;

  /**
   * Get the cost factor for A* heuristics. This factor is multiplied
   * with the distance to the destination to produce an estimate of the
   * minimum cost to the destination. The A* heuristic must underestimate the
   * cost to the destination. So a time based estimate based on speed should
   * assume the maximum speed is used to the destination such that the time
   * estimate is less than the least possible time along roads.
   */
  virtual float AStarCostFactor() const override {
    // On first pass use the walking speed plus a small factor to account for
    // favoring walkways, on the second pass use the the maximum ferry speed.
    if (pass_ == 0) {

      // Determine factor based on all of the factor options
      float factor = 1.f;
      if (walkway_factor_ < 1.f) {
        factor *= walkway_factor_;
      }
      if (sidewalk_factor_ < 1.f) {
        factor *= sidewalk_factor_;
      }
      if (alley_factor_ < 1.f) {
        factor *= alley_factor_;
      }
      if (driveway_factor_ < 1.f) {
        factor *= driveway_factor_;
      }
      if (track_factor_ < 1.f) {
        factor *= track_factor_;
      }
      if (living_street_factor_ < 1.f) {
        factor *= living_street_factor_;
      }
      if (service_factor_ < 1.f) {
        factor *= service_factor_;
      }

      return (speedfactor_ * factor);
    } else {
      return (kSecPerHour * 0.001f) / static_cast<float>(kMaxFerrySpeedKph);
    }
  }

  /**
   * Get the current travel type.
   * @return  Returns the current travel type.
   */
  virtual uint8_t travel_type() const override {
    return static_cast<uint8_t>(type_);
  }

  /**
   * Function to be used in location searching which will
   * exclude and allow ranking results from the search by looking at each
   * edges attribution and suitability for use as a location by the travel
   * mode used by the costing method. It's also used to filter
   * edges not usable / inaccessible by pedestrians.
   */
  bool Allowed(const baldr::DirectedEdge* edge,
               const graph_tile_ptr& tile,
               uint16_t disallow_mask = kDisallowNone) const override {
    return DynamicCost::Allowed(edge, tile, disallow_mask) && edge->use() < Use::kRailFerry &&
           edge->sac_scale() <= max_hiking_difficulty_ &&
           (!edge->bss_connection() || project_on_bss_connection);
  }

  virtual Cost BSSCost() const override {
    return {kDefaultBssCost, kDefaultBssPenalty};
  };

public:
  // Type: foot (default), wheelchair, etc.
  PedestrianType type_;

  // Maximum pedestrian distance.
  uint32_t max_distance_;

  // This is the factor for this mode.  The higher the value the more the
  // mode is favored.
  float mode_factor_;

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

  uint32_t max_grade_;             // Maximum grade (percent).
  SacScale max_hiking_difficulty_; // Max sac_scale (0 - 6)
  float speed_;                    // Pedestrian speed.
  float speedfactor_;              // Speed factor for costing. Based on speed.
  float walkway_factor_;           // Factor for favoring walkways and paths.
  float sidewalk_factor_;          // Factor for favoring sidewalks.
  float alley_factor_;             // Avoid alleys factor.
  float driveway_factor_;          // Avoid driveways factor.
  float step_penalty_;             // Penalty applied to steps/stairs (seconds).

  // Used in edgefilter, it tells if the location should be projected on a edge which is
  // a bike share station connection
  bool project_on_bss_connection = 0;

  /**
   * Override the base transition cost to not add maneuver penalties onto transit edges.
   * Base transition cost that all costing methods use. Includes costs for
   * country crossing, boarding a ferry, toll booth, gates, entering destination
   * only, alleys, and maneuver penalties. Each costing method can provide different
   * costs for these transitions (via costing options).
   *
   * The template allows us to treat edgelabels and directed edges the same. The unidirectinal
   * path algorithms dont have access to the directededge from the label but they have the same
   * function names. At the moment we could change edgelabel to keep the edge pointer because
   * we dont clear tiles while the algorithm is running but for embedded use-cases we might one day
   * do that so its best to keep support for labels and edges here
   *
   * @param node Node at the intersection where the edge transition occurs.
   * @param edge Directed edge entering.
   * @param pred Predecessor edge.
   * @param idx  Index used for name consistency.
   * @return Returns the transition cost (cost, elapsed time).
   */
  template <typename predecessor_t>
  sif::Cost base_transition_cost(const baldr::NodeInfo* node,
                                 const baldr::DirectedEdge* edge,
                                 const predecessor_t* pred,
                                 const uint32_t idx) const {
    // Cases with both time and penalty: country crossing, ferry, gate, toll booth
    sif::Cost c;
    c += country_crossing_cost_ * (node->type() == baldr::NodeType::kBorderControl);
    c += gate_cost_ * (node->type() == baldr::NodeType::kGate);
    c += bike_share_cost_ * (node->type() == baldr::NodeType::kBikeShare);
    c += toll_booth_cost_ * (node->type() == baldr::NodeType::kTollBooth);
    c += ferry_transition_cost_ *
         (edge->use() == baldr::Use::kFerry && pred->use() != baldr::Use::kFerry);
    c += rail_ferry_transition_cost_ *
         (edge->use() == baldr::Use::kRailFerry && pred->use() != baldr::Use::kRailFerry);

    // Additional penalties without any time cost
    c.cost += destination_only_penalty_ * (edge->destonly() && !pred->destonly());
    c.cost +=
        alley_penalty_ * (edge->use() == baldr::Use::kAlley && pred->use() != baldr::Use::kAlley);
    c.cost +=
        maneuver_penalty_ * (!edge->link() && edge->use() != Use::kEgressConnection &&
                             edge->use() != Use::kPlatformConnection && !edge->name_consistency(idx));
    c.cost += service_penalty_ *
              (edge->use() == baldr::Use::kServiceRoad && pred->use() != baldr::Use::kServiceRoad);

    // shortest ignores any penalties in favor of path length
    c.cost *= !shortest_;
    return c;
  }
};

// Constructor. Parse pedestrian options from property tree. If option is
// not present, set the default.
PedestrianCost::PedestrianCost(const CostingOptions& costing_options)
    : DynamicCost(costing_options, TravelMode::kPedestrian, kPedestrianAccess) {
  // Set hierarchy to allow unlimited transitions
  for (auto& h : hierarchy_limits_) {
    h.max_up_transitions = kUnlimitedTransitions;
  }

  allow_transit_connections_ = false;

  // Get the base costs
  get_base_costs(costing_options);

  // Get the pedestrian type - enter as string and convert to enum
  const std::string& type = costing_options.transport_type();
  if (type == "wheelchair") {
    type_ = PedestrianType::kWheelchair;
  } else if (type == "segway") {
    type_ = PedestrianType::kSegway;
  } else {
    type_ = PedestrianType::kFoot;
  }

  // Set type specific defaults, override with URL inputs
  if (type_ == PedestrianType::kWheelchair) {
    access_mask_ = kWheelchairAccess;
    minimal_allowed_surface_ = Surface::kCompacted;
  } else {
    // Assume type = foot
    access_mask_ = kPedestrianAccess;
    minimal_allowed_surface_ = Surface::kPath;
  }
  max_distance_ = costing_options.max_distance();
  speed_ = costing_options.walking_speed();
  step_penalty_ = costing_options.step_penalty();
  max_grade_ = costing_options.max_grade();

  if (type_ == PedestrianType::kFoot) {
    max_hiking_difficulty_ = static_cast<SacScale>(costing_options.max_hiking_difficulty());
  } else {
    max_hiking_difficulty_ = SacScale::kNone;
  }

  mode_factor_ = costing_options.mode_factor();
  walkway_factor_ = costing_options.walkway_factor();
  sidewalk_factor_ = costing_options.sidewalk_factor();
  alley_factor_ = costing_options.alley_factor();
  driveway_factor_ = costing_options.driveway_factor();
  transit_start_end_max_distance_ = costing_options.transit_start_end_max_distance();
  transit_transfer_max_distance_ = costing_options.transit_transfer_max_distance();

  // Set the speed factor (to avoid division in costing)
  speedfactor_ = (kSecPerHour * 0.001f) / speed_;
}

// Check if access is allowed on the specified edge. Disallow if no
// access for this pedestrian type, if surface type exceeds (worse than)
// the minimum allowed surface type, or if max grade is exceeded.
// Disallow edges where max. distance will be exceeded.
bool PedestrianCost::Allowed(const baldr::DirectedEdge* edge,
                             const EdgeLabel& pred,
                             const graph_tile_ptr& tile,
                             const baldr::GraphId& edgeid,
                             const uint64_t current_time,
                             const uint32_t tz_index,
                             uint8_t& restriction_idx) const {
  if (!IsAccessible(edge) || (edge->surface() > minimal_allowed_surface_) || edge->is_shortcut() ||
      IsUserAvoidEdge(edgeid) || edge->sac_scale() > max_hiking_difficulty_ ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx() &&
       pred.mode() == TravelMode::kPedestrian) ||
      //      (edge->max_up_slope() > max_grade_ || edge->max_down_slope() > max_grade_) ||
      ((pred.path_distance() + edge->length()) > max_distance_)) {
    return false;
  }
  // Disallow transit connections (except when set for multi-modal routes)
  if (!allow_transit_connections_ &&
      (edge->use() == Use::kPlatformConnection || edge->use() == Use::kEgressConnection ||
       edge->use() == Use::kTransitConnection)) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, tile, edgeid, current_time, tz_index,
                                           restriction_idx);
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool PedestrianCost::AllowedReverse(const baldr::DirectedEdge* edge,
                                    const EdgeLabel& pred,
                                    const baldr::DirectedEdge* opp_edge,
                                    const graph_tile_ptr& tile,
                                    const baldr::GraphId& opp_edgeid,
                                    const uint64_t current_time,
                                    const uint32_t tz_index,
                                    uint8_t& restriction_idx) const {
  // Do not check max walking distance and assume we are not allowing
  // transit connections. Assume this method is never used in
  // multimodal routes).
  if (!IsAccessible(opp_edge) || (opp_edge->surface() > minimal_allowed_surface_) ||
      opp_edge->is_shortcut() || IsUserAvoidEdge(opp_edgeid) ||
      edge->sac_scale() > max_hiking_difficulty_ ||
      (!pred.deadend() && pred.opp_local_idx() == edge->localedgeidx() &&
       pred.mode() == TravelMode::kPedestrian) ||
      //      (opp_edge->max_up_slope() > max_grade_ || opp_edge->max_down_slope() > max_grade_) ||
      opp_edge->use() == Use::kTransitConnection || opp_edge->use() == Use::kEgressConnection ||
      opp_edge->use() == Use::kPlatformConnection) {
    return false;
  }

  return DynamicCost::EvaluateRestrictions(access_mask_, edge, tile, opp_edgeid, current_time,
                                           tz_index, restriction_idx);
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost PedestrianCost::EdgeCost(const baldr::DirectedEdge* edge,
                              const graph_tile_ptr& tile,
                              const uint32_t seconds,
                              uint8_t& flow_sources) const {

  // Ferries are a special case - they use the ferry speed (stored on the edge)
  if (edge->use() == Use::kFerry) {
    auto speed = tile->GetSpeed(edge, flow_mask_, seconds, false, &flow_sources);
    float sec = edge->length() * (kSecPerHour * 0.001f) / static_cast<float>(speed);
    return {sec * ferry_factor_, sec};
  }

  float sec =
      edge->length() * speedfactor_ * kSacScaleSpeedFactor[static_cast<uint8_t>(edge->sac_scale())];

  if (shortest_) {
    return Cost(edge->length(), sec);
  }

  // TODO - consider using an array of "use factors" to avoid this conditional
  float factor = 1.0f + kSacScaleCostFactor[static_cast<uint8_t>(edge->sac_scale())];
  if (edge->use() == Use::kFootway || edge->use() == Use::kSidewalk) {
    factor *= walkway_factor_;
  } else if (edge->use() == Use::kAlley) {
    factor *= alley_factor_;
  } else if (edge->use() == Use::kDriveway) {
    factor *= driveway_factor_;
  } else if (edge->use() == Use::kTrack) {
    factor *= track_factor_;
  } else if (edge->use() == Use::kLivingStreet) {
    factor *= living_street_factor_;
  } else if (edge->use() == Use::kServiceRoad) {
    factor *= service_factor_;
  } else if (edge->sidewalk_left() || edge->sidewalk_right()) {
    factor *= sidewalk_factor_;
  } else if (edge->roundabout()) {
    factor *= kRoundaboutFactor;
  }

  // Slightly favor walkways/paths and penalize alleys and driveways.
  return {sec * factor, sec};
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost PedestrianCost::TransitionCost(const baldr::DirectedEdge* edge,
                                    const baldr::NodeInfo* node,
                                    const EdgeLabel& pred) const {
  // Special cases: fixed penalty for steps/stairs
  if (edge->use() == Use::kSteps) {
    return {step_penalty_, 0.0f};
  }

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  uint32_t idx = pred.opp_local_idx();
  Cost c = base_transition_cost(node, edge, &pred, idx);

  // Costs for crossing an intersection.
  if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
    float seconds = kCrossingCosts[edge->stopimpact(idx)];
    c.secs += seconds;
    c.cost += shortest_ ? 0.f : seconds;
  }
  return c;
}

// Returns the cost to make the transition from the predecessor edge
// when using a reverse search (from destination towards the origin).
// Defaults to 0. Costing models that wish to include edge transition
// costs (i.e., intersection/turn costs) must override this method.
Cost PedestrianCost::TransitionCostReverse(const uint32_t idx,
                                           const baldr::NodeInfo* node,
                                           const baldr::DirectedEdge* pred,
                                           const baldr::DirectedEdge* edge,
                                           const bool /*has_measured_speed*/) const {

  // TODO: do we want to update the cost if we have flow or speed from traffic.

  // Special cases: fixed penalty for steps/stairs
  if (edge->use() == Use::kSteps) {
    return {step_penalty_, 0.0f};
  }

  // Get the transition cost for country crossing, ferry, gate, toll booth,
  // destination only, alley, maneuver penalty
  Cost c = base_transition_cost(node, edge, pred, idx);

  // Costs for crossing an intersection.
  if (edge->edge_to_right(idx) && edge->edge_to_left(idx)) {
    float seconds = kCrossingCosts[edge->stopimpact(idx)];
    c.secs += seconds;
    c.cost += shortest_ ? 0.f : seconds;
  }
  return c;
}

void ParsePedestrianCostOptions(const rapidjson::Document& doc,
                                const std::string& costing_options_key,
                                CostingOptions* pbf_costing_options) {
  pbf_costing_options->set_costing(Costing::pedestrian);
  pbf_costing_options->set_name(Costing_Enum_Name(pbf_costing_options->costing()));
  auto json_costing_options = rapidjson::get_child_optional(doc, costing_options_key.c_str());

  if (json_costing_options) {
    // TODO: farm more common stuff out to parent class
    ParseSharedCostOptions(*json_costing_options, pbf_costing_options);

    // If specified, parse json and set pbf values

    // maneuver_penalty
    pbf_costing_options->set_maneuver_penalty(kManeuverPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/maneuver_penalty")
            .get_value_or(kDefaultManeuverPenalty)));

    // destination_only_penalty
    pbf_costing_options->set_destination_only_penalty(kDestinationOnlyPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/destination_only_penalty")
            .get_value_or(kDefaultDestinationOnlyPenalty)));

    // gate_penalty
    pbf_costing_options->set_gate_penalty(
        kGatePenaltyRange(rapidjson::get_optional<float>(*json_costing_options, "/gate_penalty")
                              .get_value_or(kDefaultGatePenalty)));

    // alley_penalty
    pbf_costing_options->set_alley_penalty(
        kAlleyPenaltyRange(rapidjson::get_optional<float>(*json_costing_options, "/alley_penalty")
                               .get_value_or(kDefaultAlleyPenalty)));

    // country_crossing_cost
    pbf_costing_options->set_country_crossing_cost(kCountryCrossingCostRange(
        rapidjson::get_optional<float>(*json_costing_options, "/country_crossing_cost")
            .get_value_or(kDefaultCountryCrossingCost)));

    // country_crossing_penalty
    pbf_costing_options->set_country_crossing_penalty(kCountryCrossingPenaltyRange(
        rapidjson::get_optional<float>(*json_costing_options, "/country_crossing_penalty")
            .get_value_or(kDefaultCountryCrossingPenalty)));

    // ferry_cost
    pbf_costing_options->set_ferry_cost(
        kFerryCostRange(rapidjson::get_optional<float>(*json_costing_options, "/ferry_cost")
                            .get_value_or(kDefaultFerryCost)));

    // use_ferry
    pbf_costing_options->set_use_ferry(
        kUseFerryRange(rapidjson::get_optional<float>(*json_costing_options, "/use_ferry")
                           .get_value_or(kDefaultUseFerry)));

    // type (transport_type)
    pbf_costing_options->set_transport_type(
        (rapidjson::get_optional<std::string>(*json_costing_options, "/type").get_value_or("foot")));

    // Set type specific defaults, override with URL inputs
    if (pbf_costing_options->transport_type() == "wheelchair") {
      // max_distance
      pbf_costing_options->set_max_distance(kMaxDistanceWheelchairRange(
          rapidjson::get_optional<uint32_t>(*json_costing_options, "/max_distance")
              .get_value_or(kMaxDistanceWheelchair)));

      // walking_speed
      pbf_costing_options->set_walking_speed(kSpeedWheelchairRange(
          rapidjson::get_optional<float>(*json_costing_options, "/walking_speed")
              .get_value_or(kDefaultSpeedWheelchair)));

      // step_penalty
      pbf_costing_options->set_step_penalty(kStepPenaltyWheelchairRange(
          rapidjson::get_optional<float>(*json_costing_options, "/step_penalty")
              .get_value_or(kDefaultStepPenaltyWheelchair)));

      // max_grade
      pbf_costing_options->set_max_grade(kMaxGradeWheelchairRange(
          rapidjson::get_optional<uint32_t>(*json_costing_options, "/max_grade")
              .get_value_or(kDefaultMaxGradeWheelchair)));

    } else {
      // Assume type = foot
      // max_distance
      pbf_costing_options->set_max_distance(kMaxDistanceFootRange(
          rapidjson::get_optional<uint32_t>(*json_costing_options, "/max_distance")
              .get_value_or(kMaxDistanceFoot)));

      // walking_speed
      pbf_costing_options->set_walking_speed(
          kSpeedFootRange(rapidjson::get_optional<float>(*json_costing_options, "/walking_speed")
                              .get_value_or(kDefaultSpeedFoot)));

      // step_penalty
      pbf_costing_options->set_step_penalty(
          kStepPenaltyFootRange(rapidjson::get_optional<float>(*json_costing_options, "/step_penalty")
                                    .get_value_or(kDefaultStepPenaltyFoot)));

      // max_grade
      pbf_costing_options->set_max_grade(
          kMaxGradeFootRange(rapidjson::get_optional<uint32_t>(*json_costing_options, "/max_grade")
                                 .get_value_or(kDefaultMaxGradeFoot)));
    }

    // max_hiking_difficulty
    pbf_costing_options->set_max_hiking_difficulty(kMaxHikingDifficultyRange(
        rapidjson::get_optional<uint32_t>(*json_costing_options, "/max_hiking_difficulty")
            .get_value_or(kDefaultMaxHikingDifficulty)));

    // mode_factor
    pbf_costing_options->set_mode_factor(
        kModeFactorRange(rapidjson::get_optional<float>(*json_costing_options, "/mode_factor")
                             .get_value_or(kModeFactor)));

    // walkway_factor
    pbf_costing_options->set_walkway_factor(
        kWalkwayFactorRange(rapidjson::get_optional<float>(*json_costing_options, "/walkway_factor")
                                .get_value_or(kDefaultWalkwayFactor)));

    // sidewalk_factor
    pbf_costing_options->set_sidewalk_factor(
        kSideWalkFactorRange(rapidjson::get_optional<float>(*json_costing_options, "/sidewalk_factor")
                                 .get_value_or(kDefaultSideWalkFactor)));

    // alley_factor
    pbf_costing_options->set_alley_factor(
        kAlleyFactorRange(rapidjson::get_optional<float>(*json_costing_options, "/alley_factor")
                              .get_value_or(kDefaultAlleyFactor)));

    // driveway_factor
    pbf_costing_options->set_driveway_factor(
        kDrivewayFactorRange(rapidjson::get_optional<float>(*json_costing_options, "/driveway_factor")
                                 .get_value_or(kDefaultDrivewayFactor)));

    // transit_start_end_max_distance
    pbf_costing_options->set_transit_start_end_max_distance(kTransitStartEndMaxDistanceRange(
        rapidjson::get_optional<uint32_t>(*json_costing_options, "/transit_start_end_max_distance")
            .get_value_or(kTransitStartEndMaxDistance)));

    // transit_transfer_max_distance
    pbf_costing_options->set_transit_transfer_max_distance(kTransitTransferMaxDistanceRange(
        rapidjson::get_optional<uint32_t>(*json_costing_options, "/transit_transfer_max_distance")
            .get_value_or(kTransitTransferMaxDistance)));

    // bss rent cost
    pbf_costing_options->set_bike_share_cost(
        kBSSCostRange(rapidjson::get_optional<uint32_t>(*json_costing_options, "/bss_rent_cost")
                          .get_value_or(kDefaultBssCost)));
    pbf_costing_options->set_bike_share_penalty(
        kBSSPenaltyRange(rapidjson::get_optional<uint32_t>(*json_costing_options, "/bss_rent_penalty")
                             .get_value_or(kDefaultBssPenalty)));

    // use_tracks
    pbf_costing_options->set_use_tracks(
        kUseTracksRange(rapidjson::get_optional<float>(*json_costing_options, "/use_tracks")
                            .get_value_or(kDefaultUseTracks)));

    // use_living_street
    pbf_costing_options->set_use_living_streets(kUseLivingStreetsRange(
        rapidjson::get_optional<float>(*json_costing_options, "/use_living_streets")
            .get_value_or(kDefaultUseLivingStreets)));

    // service_penalty
    pbf_costing_options->set_service_penalty(
        kServicePenaltyRange(rapidjson::get_optional<float>(*json_costing_options, "/service_penalty")
                                 .get_value_or(kDefaultServicePenalty)));

    // service_factor
    pbf_costing_options->set_service_factor(
        kServiceFactorRange(rapidjson::get_optional<float>(*json_costing_options, "/service_factor")
                                .get_value_or(kDefaultServiceFactor)));
  } else {
    // Set pbf values to defaults
    pbf_costing_options->set_maneuver_penalty(kDefaultManeuverPenalty);
    pbf_costing_options->set_destination_only_penalty(kDefaultDestinationOnlyPenalty);
    pbf_costing_options->set_gate_penalty(kDefaultGatePenalty);
    pbf_costing_options->set_alley_penalty(kDefaultAlleyPenalty);
    pbf_costing_options->set_country_crossing_cost(kDefaultCountryCrossingCost);
    pbf_costing_options->set_country_crossing_penalty(kDefaultCountryCrossingPenalty);
    pbf_costing_options->set_ferry_cost(kDefaultFerryCost);
    pbf_costing_options->set_use_ferry(kDefaultUseFerry);
    pbf_costing_options->set_transport_type("foot");
    pbf_costing_options->set_max_distance(kMaxDistanceFoot);
    pbf_costing_options->set_walking_speed(kDefaultSpeedFoot);
    pbf_costing_options->set_step_penalty(kDefaultStepPenaltyFoot);
    pbf_costing_options->set_max_grade(kDefaultMaxGradeFoot);
    pbf_costing_options->set_max_hiking_difficulty(kDefaultMaxHikingDifficulty);
    pbf_costing_options->set_mode_factor(kModeFactor);
    pbf_costing_options->set_walkway_factor(kDefaultWalkwayFactor);
    pbf_costing_options->set_sidewalk_factor(kDefaultSideWalkFactor);
    pbf_costing_options->set_alley_factor(kDefaultAlleyFactor);
    pbf_costing_options->set_driveway_factor(kDefaultDrivewayFactor);
    pbf_costing_options->set_transit_start_end_max_distance(kTransitStartEndMaxDistance);
    pbf_costing_options->set_transit_transfer_max_distance(kTransitTransferMaxDistance);
    pbf_costing_options->set_bike_share_cost(kDefaultBssCost);
    pbf_costing_options->set_bike_share_penalty(kDefaultBssPenalty);
    pbf_costing_options->set_use_tracks(kDefaultUseTracks);
    pbf_costing_options->set_use_living_streets(kDefaultUseLivingStreets);
    pbf_costing_options->set_service_penalty(kDefaultServicePenalty);
    pbf_costing_options->set_service_factor(kDefaultServiceFactor);
  }
}

cost_ptr_t CreatePedestrianCost(const CostingOptions& costing_options) {
  return std::make_shared<PedestrianCost>(costing_options);
}

cost_ptr_t CreateBikeShareCost(const CostingOptions& costing_options) {
  auto cost_ptr = std::make_shared<PedestrianCost>(costing_options);
  cost_ptr->project_on_bss_connection = true;
  return cost_ptr;
}

} // namespace sif
} // namespace valhalla

/**********************************************************************************************/

#ifdef INLINE_TEST

using namespace valhalla;
using namespace sif;

namespace {

class TestPedestrianCost : public PedestrianCost {
public:
  TestPedestrianCost(const CostingOptions& costing_options) : PedestrianCost(costing_options){};

  using PedestrianCost::alley_penalty_;
  using PedestrianCost::country_crossing_cost_;
  using PedestrianCost::destination_only_penalty_;
  using PedestrianCost::ferry_transition_cost_;
  using PedestrianCost::gate_cost_;
  using PedestrianCost::maneuver_penalty_;
  using PedestrianCost::service_factor_;
  using PedestrianCost::service_penalty_;
};

TestPedestrianCost* make_pedestriancost_from_json(const std::string& property,
                                                  float testVal,
                                                  const std::string& /*type*/) {
  std::stringstream ss;
  ss << R"({"costing_options":{"pedestrian":{")" << property << R"(":)" << testVal << "}}}";
  Api request;
  ParseApi(ss.str(), valhalla::Options::route, request);
  return new TestPedestrianCost(
      request.options().costing_options(static_cast<int>(Costing::pedestrian)));
}

std::uniform_real_distribution<float>*
make_distributor_from_range(const ranged_default_t<float>& range) {
  float rangeLength = range.max - range.min;
  return new std::uniform_real_distribution<float>(range.min - rangeLength, range.max + rangeLength);
}

std::uniform_int_distribution<uint32_t>*
make_distributor_from_range(const ranged_default_t<uint32_t>& range) {
  uint32_t rangeLength = range.max - range.min;
  return new std::uniform_int_distribution<uint32_t>(range.min - rangeLength,
                                                     range.max + rangeLength);
}

TEST(PedestrianCost, testPedestrianCostParams) {
  constexpr unsigned testIterations = 250;
  constexpr unsigned seed = 0;
  std::mt19937 generator(seed);
  std::shared_ptr<std::uniform_real_distribution<float>> real_distributor;
  std::shared_ptr<std::uniform_int_distribution<uint32_t>> int_distributor;
  std::shared_ptr<TestPedestrianCost> ctorTester;

  // maneuver_penalty_
  real_distributor.reset(make_distributor_from_range(kManeuverPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("maneuver_penalty", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->maneuver_penalty_,
                test::IsBetween(kManeuverPenaltyRange.min, kManeuverPenaltyRange.max));
  }

  // gate_penalty_
  real_distributor.reset(make_distributor_from_range(kGatePenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("gate_penalty", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->gate_cost_.cost,
                test::IsBetween(kGatePenaltyRange.min, kGatePenaltyRange.max));
  }

  // alley_factor_
  real_distributor.reset(make_distributor_from_range(kAlleyFactorRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("alley_factor", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->alley_factor_,
                test::IsBetween(kAlleyFactorRange.min, kAlleyFactorRange.max));
  }

  // ferry_cost_
  real_distributor.reset(make_distributor_from_range(kFerryCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("ferry_cost", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->ferry_transition_cost_.secs,
                test::IsBetween(kFerryCostRange.min, kFerryCostRange.max));
  }

  // country_crossing_cost_
  real_distributor.reset(make_distributor_from_range(kCountryCrossingCostRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_pedestriancost_from_json("country_crossing_cost",
                                                   (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->country_crossing_cost_.secs,
                test::IsBetween(kCountryCrossingCostRange.min, kCountryCrossingCostRange.max));
  }

  // country_crossing_penalty_
  real_distributor.reset(make_distributor_from_range(kCountryCrossingPenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_pedestriancost_from_json("country_crossing_penalty",
                                                   (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->country_crossing_cost_.cost,
                test::IsBetween(kCountryCrossingPenaltyRange.min,
                                kCountryCrossingPenaltyRange.max + kDefaultCountryCrossingCost));
  }

  // Wheelchair tests
  // max_distance_
  int_distributor.reset(make_distributor_from_range(kMaxDistanceWheelchairRange));
  for (unsigned i = 0; i < 100; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("max_distance", (*int_distributor)(generator), "wheelchair"));
    EXPECT_THAT(ctorTester->max_distance_,
                test::IsBetween(kMaxDistanceWheelchairRange.min, kMaxDistanceWheelchairRange.max));
  }

  // speed_
  real_distributor.reset(make_distributor_from_range(kSpeedWheelchairRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("walking_speed", (*real_distributor)(generator), "wheelchair"));
    EXPECT_THAT(ctorTester->speed_,
                test::IsBetween(kSpeedWheelchairRange.min, kSpeedWheelchairRange.max));
  }

  // step_penalty_
  real_distributor.reset(make_distributor_from_range(kStepPenaltyWheelchairRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("step_penalty", (*real_distributor)(generator), "wheelchair"));
    EXPECT_THAT(ctorTester->step_penalty_,
                test::IsBetween(kStepPenaltyWheelchairRange.min, kStepPenaltyWheelchairRange.max));
  }

  // max_grade_
  int_distributor.reset(make_distributor_from_range(kMaxGradeWheelchairRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("max_grade", (*int_distributor)(generator), "wheelchair"));
    EXPECT_THAT(ctorTester->max_grade_,
                test::IsBetween(kMaxGradeWheelchairRange.min, kMaxGradeWheelchairRange.max));
  }

  // Foot tests
  // max_distance_
  int_distributor.reset(make_distributor_from_range(kMaxDistanceFootRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("max_distance", (*int_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->max_distance_,
                test::IsBetween(kMaxDistanceFootRange.min, kMaxDistanceFootRange.max));
  }

  // speed_
  real_distributor.reset(make_distributor_from_range(kSpeedFootRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("walking_speed", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->speed_, test::IsBetween(kSpeedFootRange.min, kSpeedFootRange.max));
  }

  // step_penalty_
  real_distributor.reset(make_distributor_from_range(kStepPenaltyFootRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("step_penalty", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->step_penalty_,
                test::IsBetween(kStepPenaltyFootRange.min, kStepPenaltyFootRange.max));
  }

  // max_grade_
  int_distributor.reset(make_distributor_from_range(kMaxGradeFootRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("max_grade", (*int_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->max_grade_,
                test::IsBetween(kMaxGradeFootRange.min, kMaxGradeFootRange.max));
  }

  // Non type dependent tests
  // mode_factor_
  real_distributor.reset(make_distributor_from_range(kModeFactorRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("mode_factor", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->mode_factor_,
                test::IsBetween(kModeFactorRange.min, kModeFactorRange.max));
  }

  /*
   // use_ferry_
   real_distributor.reset(make_distributor_from_range(kUseFerryRange));
   for (unsigned i = 0; i < testIterations; ++i) {
     ctorTester.reset(
         make_pedestriancost_from_json("use_ferry", (*real_distributor)(generator), "foot"));
EXPECT_THAT(ctorTester->use_ferry_ , test::IsBetween( kUseFerryRange.min ,kUseFerryRange.max));
   }

   */

  // walkway_factor_
  real_distributor.reset(make_distributor_from_range(kWalkwayFactorRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("walkway_factor", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->walkway_factor_,
                test::IsBetween(kWalkwayFactorRange.min, kWalkwayFactorRange.max));
  }

  // sidewalk_factor_
  real_distributor.reset(make_distributor_from_range(kSideWalkFactorRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("sidewalk_factor", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->sidewalk_factor_,
                test::IsBetween(kSideWalkFactorRange.min, kSideWalkFactorRange.max));
  }

  // driveway_factor_
  real_distributor.reset(make_distributor_from_range(kDrivewayFactorRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("driveway_factor", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->driveway_factor_,
                test::IsBetween(kDrivewayFactorRange.min, kDrivewayFactorRange.max));
  }

  // transit_start_end_max_distance_
  int_distributor.reset(make_distributor_from_range(kTransitStartEndMaxDistanceRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_pedestriancost_from_json("transit_start_end_max_distance",
                                                   (*int_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->transit_start_end_max_distance_,
                test::IsBetween(kTransitStartEndMaxDistanceRange.min,
                                kTransitStartEndMaxDistanceRange.max));
  }

  // transit_transfer_max_distance_
  int_distributor.reset(make_distributor_from_range(kTransitTransferMaxDistanceRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(make_pedestriancost_from_json("transit_transfer_max_distance",
                                                   (*int_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->transit_transfer_max_distance_,
                test::IsBetween(kTransitTransferMaxDistanceRange.min,
                                kTransitTransferMaxDistanceRange.max));
  }

  // service_penalty_
  real_distributor.reset(make_distributor_from_range(kServicePenaltyRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("service_penalty", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->service_penalty_,
                test::IsBetween(kServicePenaltyRange.min, kServicePenaltyRange.max));
  }

  // service_factor_
  real_distributor.reset(make_distributor_from_range(kServiceFactorRange));
  for (unsigned i = 0; i < testIterations; ++i) {
    ctorTester.reset(
        make_pedestriancost_from_json("service_factor", (*real_distributor)(generator), "foot"));
    EXPECT_THAT(ctorTester->service_factor_,
                test::IsBetween(kServiceFactorRange.min, kServiceFactorRange.max));
  }
}
} // namespace

int main(int argc, char* argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#endif
