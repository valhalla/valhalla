#include "sif/transitcost.h"

#include <valhalla/baldr/accessrestriction.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;

namespace valhalla {
namespace sif {

// Default options/values
namespace {
constexpr uint32_t kUnitSize = 1;

constexpr float kModeWeight             = 1.0f; // Favor this mode?
constexpr float kDefaultTransferCost    = 15.0f;
constexpr float kDefaultTransferPenalty = 300.0f;

// User propensity to use buses. Range of values from 0 (avoid buses) to
// 1 (totally comfortable riding on buses).
constexpr float kDefaultUseBusFactor = 0.3f;

// User propensity to use rail. Range of values from 0 (avoid rail) to
// 1 (totally comfortable riding on rail).
constexpr float kDefaultUseRailFactor = 0.6f;

// User propensity to use/allow transfers. Range of values from 0
// (avoid transfers) to 1 (totally comfortable with transfers).
constexpr float kDefaultUseTransfersFactor = 0.3f;

Cost kImpossibleCost = { 10000000.0f, 10000000.0f };
}

/**
 * Derived class providing dynamic edge costing for transit parts
 * of multi-modal routes.
 */
class TransitCost : public DynamicCost {
 public:
  /**
   * Constructor. Configuration / options for pedestrian costing are provided
   * via a property tree (JSON).
   * @param  pt  Property tree with configuration/options.
   */
  TransitCost(const boost::property_tree::ptree& pt);

  virtual ~TransitCost();

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
   * @param  opp_pred_edge  Pointer to the opposing directed edge to the
   *                        predecessor.
   * @param  tile           current tile
   * @param  edgeid         edgeid that we care about
   * @return  Returns true if access is allowed, false if not.
   */
  virtual bool AllowedReverse(const baldr::DirectedEdge* edge,
                 const EdgeLabel& pred,
                 const baldr::DirectedEdge* opp_edge,
                 const baldr::DirectedEdge* opp_pred_edge,
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
   * Get the cost to traverse the specified directed edge using a transit
   * departure (schedule based edge traversal). Cost includes
   * the time (seconds) to traverse the edge.
   * @param   edge      Pointer to a directed edge.
   * @param   departure Transit departure record.
   * @param   curr_time Current local time (seconds from midnight).
   * @return  Returns the cost and time (seconds)
   */
  virtual Cost EdgeCost(const baldr::DirectedEdge* edge,
                        const baldr::TransitDeparture* departure,
                        const uint32_t curr_time) const;

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
   * Returns the transfer cost between 2 transit stops.
   * @return  Returns the transfer cost and time (seconds).
   */
  virtual Cost TransferCost() const;

  /**
   * Returns the default transfer cost between 2 transit lines.
   * @return  Returns the transfer cost and time (seconds).
   */
  virtual Cost DefaultTransferCost() const;

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
   * Override unit size since walking costs are higher range of vales
   */
  virtual uint32_t UnitSize() const;

  /**
   * Returns a function/functor to be used in location searching which will
   * exclude results from the search by looking at each edges attribution
   * @return Function/functor to be used in filtering out edges
   */
  virtual const EdgeFilter GetFilter() const {
    //throw back a lambda that checks the access for this type of costing
    // Disallow transit edges as start/end of a route leg
    return [](const baldr::DirectedEdge* edge){
      return false;
    };
  }

 protected:

  // This is the weight for this mode.  The higher the value the more the
  // mode is favored.
  float mode_weight_;

  // A measure of willingness to ride on buses or rail. Ranges from 0-1 with
  // 0 being not willing at all and 1 being totally comfortable with taking
  // this transportation. These factors determine how much rail or buses are
  // preferred over each other (if at all).
  float use_bus_;
  float use_rail_;
  float bus_factor_;
  float rail_factor_;

  // A measure of willingness to make transfers. Ranges from 0-1 with
  // 0 being not willing at all and 1 being totally comfortable.
  float use_transfers_;
  float transfer_factor_;

  float transfer_cost_;     // Transfer cost
  float transfer_penalty_;  // Transfer penalty
};

// Constructor. Parse pedestrian options from property tree. If option is
// not present, set the default.
TransitCost::TransitCost(const boost::property_tree::ptree& pt)
    : DynamicCost(pt, TravelMode::kPublicTransit) {

  mode_weight_ = pt.get<float>("mode_weight", kModeWeight);

  // Willingness to use buses. Make sure this is within range [0, 1].
  use_bus_ = pt.get<float>("use_bus", kDefaultUseBusFactor);
  if (use_bus_ < 0.0f || use_bus_ > 1.0f) {
    use_bus_ = kDefaultUseBusFactor;
    LOG_WARN("Outside valid use_bus factor range " +
              std::to_string(use_bus_) + ": using default");
  }

  // Willingness to use rail. Make sure this is within range [0, 1].
  use_rail_ = pt.get<float>("use_rail", kDefaultUseRailFactor);
  if (use_rail_ < 0.0f || use_rail_ > 1.0f) {
    use_rail_ = kDefaultUseRailFactor;
    LOG_WARN("Outside valid use_rail factor range " +
              std::to_string(use_rail_) + ": using default");
  }

  // Willingness to make transfers. Make sure this is within range [0, 1].
  use_transfers_ = pt.get<float>("use_transfers", kDefaultUseTransfersFactor);
  if (use_transfers_ < 0.0f || use_transfers_ > 1.0f) {
    use_transfers_ = kDefaultUseTransfersFactor;
    LOG_WARN("Outside valid use_transfers factor range " +
              std::to_string(use_transfers_) + ": using default");
  }

  // Set the factors. The factors above 0.5 start to reduce the weight
  // for this mode while factors below 0.5 start to increase the weight for
  // this mode.
  bus_factor_ = (use_bus_ >= 0.5f) ?
                 1.0f - (use_bus_ - 0.5f) :
                 1.0f + (0.5f - use_bus_) * 5.0f;

  rail_factor_ = (use_rail_ >= 0.5f) ?
                 1.0f - (use_rail_ - 0.5f) :
                 1.0f + (0.5f - use_rail_) * 5.0f;

  transfer_factor_ = (use_transfers_ >= 0.5f) ?
                     1.0f - (use_transfers_ - 0.5f) :
                     1.0f + (0.5f - use_transfers_) * 5.0f;

  transfer_cost_ = pt.get<float>("transfer_cost", kDefaultTransferCost);
  transfer_penalty_ = pt.get<float>("transfer_penalty", kDefaultTransferPenalty);

  // Normalize so the favored mode has factor == 1
  if (rail_factor_ < bus_factor_) {
    float ratio = bus_factor_ / rail_factor_;
    rail_factor_ = 1.0f;
    bus_factor_ *= ratio;
  } else {
    float ratio = rail_factor_ / bus_factor_;
    bus_factor_ = 1.0f;
    rail_factor_ *= ratio;
  }
}

// Destructor
TransitCost::~TransitCost() {
}

// This method overrides the weight for this mode.  The higher the value
// the more the mode is favored.
float TransitCost::GetModeWeight() {
  return mode_weight_;
}

// Check if access is allowed on the specified edge.
bool TransitCost::Allowed(const baldr::DirectedEdge* edge,
                          const EdgeLabel& pred,
                          const baldr::GraphTile*& tile,
                          const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  if (edge->use() == Use::kBus) {
    return (use_bus_ > 0.0f) ? true : false;
  } else if (edge->use() == Use::kRail) {
    return (use_rail_ > 0.0f) ? true : false;
  }
  return true;
}

// Checks if access is allowed for an edge on the reverse path (from
// destination towards origin). Both opposing edges are provided.
bool TransitCost::AllowedReverse(const baldr::DirectedEdge* edge,
               const EdgeLabel& pred,
               const baldr::DirectedEdge* opp_edge,
               const baldr::DirectedEdge* opp_pred_edge,
               const baldr::GraphTile*& tile,
               const baldr::GraphId& edgeid) const {
  // TODO - obtain and check the access restrictions.

  // This method should not be called since time based routes do not use
  // bidirectional A*
  return false;
}

// Check if access is allowed at the specified node.
bool TransitCost::Allowed(const baldr::NodeInfo* node) const {
  return true;
}

// Returns the cost to traverse the edge and an estimate of the actual time
// (in seconds) to traverse the edge.
Cost TransitCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const uint32_t density) const {
  LOG_ERROR("Wrong transit edge cost called");
  return { 0.0f, 0.0f };
}

// Get the cost to traverse the specified directed edge using a transit
// departure (schedule based edge traversal). Cost includes
// the time (seconds) to traverse the edge. Only transit cost models override
// this method.
Cost TransitCost::EdgeCost(const baldr::DirectedEdge* edge,
                           const baldr::TransitDeparture* departure,
                           const uint32_t curr_time) const {
  // Separate wait time from time on transit
  float wait_time = departure->departure_time() - curr_time;

  // Cost is modulated by mode-based weight factor
  float weight = 1.0f;
  if (edge->use() == Use::kBus) {
    weight *= bus_factor_;
  } else if (edge->use() == Use::kRail) {
    weight *= rail_factor_;
  }
  return { wait_time + (departure->elapsed_time() * weight),
           wait_time + departure->elapsed_time() };
}

// Returns the time (in seconds) to make the transition from the predecessor
Cost TransitCost::TransitionCost(const baldr::DirectedEdge* edge,
                                 const baldr::NodeInfo* node,
                                 const EdgeLabel& pred) const {
  if (pred.mode() == TravelMode::kPedestrian) {
    // Apply any mode-based penalties when boarding transit
    // Do we want any time cost to board?
    if (edge->use() == Use::kBus) {
      return { (0.5f + bus_factor_), 0.0f };
    } else if (edge->use() == Use::kRail) {
      return { (0.5f + rail_factor_), 0.0f };
    }
  }
  return { 0.0f, 0.0f };
}

// Returns the transfer cost between 2 transit stops.
Cost TransitCost::TransferCost() const {
  // Defaults...15 seconds for in station transfer and 1 minute otherwise
  return { (transfer_cost_ +  transfer_penalty_) * transfer_factor_,
            transfer_cost_ * 4.0f};
}

// Returns the default transfer cost between 2 transit lines.
Cost TransitCost::DefaultTransferCost() const {
  return { transfer_cost_ +  transfer_penalty_ , transfer_cost_ };
}

// Get the cost factor for A* heuristics. This factor is multiplied
// with the distance to the destination to produce an estimate of the
// minimum cost to the destination. The A* heuristic must underestimate the
// cost to the destination. So a time based estimate based on speed should
// assume the maximum speed is used to the destination such that the time
// estimate is less than the least possible time along roads.
float TransitCost::AStarCostFactor() const {
  return 0.0f;
}

//  Override unit size since walking costs are higher range of values
uint32_t TransitCost::UnitSize() const {
  return kUnitSize;
}

cost_ptr_t CreateTransitCost(const boost::property_tree::ptree& config) {
  return std::make_shared<TransitCost>(config);
}

}
}
